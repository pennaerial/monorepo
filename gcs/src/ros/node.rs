use crate::ros::server_types::{CallId, Channel, ChannelId, ServerMessage, Service, ServiceId};
use crate::ros::subscription::DeliveryError;
use crate::ros::SubscriptionSink;
use crate::ros::{
    foxglove_client::FoxgloveClientError, node_command::NodeCommand, FoxgloveClient,
    SubscriptionHandle, SubscriptionId, SubscriptionSinkBase,
};
use dioxus::logger::tracing::{debug, error, info, warn};
use ros_interfaces::{RosMessage, RosService};
use std::collections::HashMap;
use std::fmt;
use std::time::Duration;
use tokio::sync::{mpsc, oneshot};
use tungstenite::Bytes;

pub type NodeCommandReceiver = mpsc::Receiver<NodeCommand>;
pub type NodeCommandSender = mpsc::Sender<NodeCommand>;

/// Encoding used for service requests when the server doesn't advertise one
const DEFAULT_SERVICE_ENCODING: &str = "cdr";

// TODO: Look into maybe splitting these error types into ServiceErrors, SubscriptionErrors, etc. etc.

#[derive(Debug)]
pub enum NodeError {
    NodeStopped,
    Foxglove(FoxgloveClientError),
    ServiceNotAdvertised(String),
    ServiceTypeMismatch {
        expected: String,
        advertised: String,
    },
    ServiceCallFailed(String),
    ServiceUnavailable,
    Timeout,
    Encode(cdr::Error),
    Decode(cdr::Error),
}

impl fmt::Display for NodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NodeStopped => write!(f, "ROS node is not running"),
            Self::Foxglove(error) => write!(f, "{error}"),
            Self::ServiceNotAdvertised(name) => write!(f, "service {name} is not advertised"),
            Self::ServiceTypeMismatch {
                expected,
                advertised,
            } => write!(
                f,
                "service type mismatch: expected {expected}, server advertises {advertised}"
            ),
            Self::ServiceCallFailed(message) => write!(f, "service call failed: {message}"),
            Self::ServiceUnavailable => write!(f, "service went away before responding"),
            Self::Timeout => write!(f, "timed out waiting for a response"),
            Self::Encode(error) => write!(f, "failed to encode request: {error}"),
            Self::Decode(error) => write!(f, "failed to decode response: {error}"),
        }
    }
}

/// A service call waiting for its response from the server
struct PendingServiceCall {
    service_id: ServiceId,
    reply: oneshot::Sender<Result<Bytes, NodeError>>,
}

/// stores all necessary data for a pending subscription
struct PendingSubscription {
    sink: Box<dyn SubscriptionSinkBase>,
    reply: oneshot::Sender<Result<SubscriptionId, NodeError>>,
}

pub struct RosNode {
    foxglove_client: FoxgloveClient,
    /// client allows only one subscription per channelId
    subscriptions: HashMap<SubscriptionId, Vec<Box<dyn SubscriptionSinkBase>>>,
    /// for subscription on topics that haven't been advertised yet
    pending_subscribes: HashMap<String, Vec<PendingSubscription>>,
    /// for receiving NodeCommands sent from a NodeHandle
    node_command_receiver: NodeCommandReceiver,
    /// Tracked advertised channels from FoxgloveClient
    advertised_channels: HashMap<ChannelId, Channel>,
    /// Tracked advertised services from FoxgloveClient
    advertised_services: HashMap<ServiceId, Service>,
    /// Service calls sent to the server and awaiting a response
    pending_service_calls: HashMap<CallId, PendingServiceCall>,
}

impl RosNode {
    pub fn new(command_channel_depth: usize) -> (NodeHandle, Self) {
        let (sender, receiver) = mpsc::channel(command_channel_depth);
        let handle = NodeHandle {
            node_command_sender: sender,
        };
        let node = Self {
            foxglove_client: FoxgloveClient::new(),
            subscriptions: HashMap::new(),
            pending_subscribes: HashMap::new(),
            node_command_receiver: receiver,
            advertised_channels: HashMap::new(),
            advertised_services: HashMap::new(),
            pending_service_calls: HashMap::new(),
        };
        (handle, node)
    }

    /// Runs the node's receive loop and callbacks
    pub async fn spin(&mut self) -> Result<(), NodeError> {
        let mut server_receiver = self.foxglove_client.get_receiver();
        debug!("Node spinning");
        loop {
            tokio::select! {
                command = self.node_command_receiver.recv() => {
                    let Some(command) = command else {
                        break;
                    };
                    self.handle_command(command).await;
                }

                message = server_receiver.recv() => {
                    match message {
                        Ok(server_message) => self.handle_server_message(server_message).await,
                        Err(e) => error!("{e}"),
                    }
                }
            }
        }
        Ok(())
    }

    async fn handle_command(&mut self, command: NodeCommand) {
        match command {
            NodeCommand::Connect { url, reply } => {
                debug!("connect command received");
                let result = if self.foxglove_client.is_connected() {
                    Ok(()) // handle already connected case
                } else {
                    self.foxglove_client
                        .connect(&url)
                        .await
                        .map_err(NodeError::Foxglove)
                };
                if reply.send(result).is_err() {
                    debug!("Connect caller dropped response receiver");
                }
            }
            NodeCommand::CreateSubscription { sink, reply } => {
                let topic = sink.topic_name();

                // None if no corresponding channel_id exists for the topic name
                let channel_id = self
                    .advertised_channels
                    .values()
                    .find(|channel| channel.topic == topic)
                    .map(|channel| channel.id);

                // If channel exists, then send subscribe request through foxglove client
                if let Some(channel_id) = channel_id {
                    self.activate_subscription(channel_id, sink, reply).await;
                } else {
                    debug!(
                        "no channel detected for {}, creating pending subscription",
                        topic
                    );
                    // if no channel_id matches, save it to pending_subscribes for later
                    // don't send the reply yet bc no channel_id has been detected
                    self.pending_subscribes
                        .entry(topic.to_owned())
                        .or_default()
                        .push(PendingSubscription { sink, reply });
                }
            }
            // TODO: Skip sending if reply.is_closed(): the caller may have timed out
            // while this command was queued. Also clean up timed-out pending calls
            // without waiting for another call or response.
            NodeCommand::CallService {
                service_name,
                expected_type,
                request,
                reply,
            } => match self
                .send_service_call(&service_name, expected_type, &request)
                .await
            {
                Ok((call_id, service_id)) => {
                    // drop calls whose callers have given up (timed out) and never got a response
                    self.pending_service_calls
                        .retain(|_, call| !call.reply.is_closed());
                    self.pending_service_calls
                        .insert(call_id, PendingServiceCall { service_id, reply });
                }
                Err(e) => {
                    if reply.send(Err(e)).is_err() {
                        debug!("CallService caller dropped response receiver");
                    }
                }
            },
            _ => (),
        }
    }

    /// subscribes over foxglove, storing sink and sending result
    async fn activate_subscription(
        &mut self,
        channel_id: ChannelId,
        sink: Box<dyn SubscriptionSinkBase>,
        reply: oneshot::Sender<Result<SubscriptionId, NodeError>>,
    ) {
        let result = self
            .foxglove_client
            .subscribe(channel_id)
            .await
            .map_err(NodeError::Foxglove);

        // Keep the sink alive if subscription creation succeeded.
        if let Ok(id) = &result {
            self.add_sink(*id, sink);
        }

        if reply.send(result).is_err() {
            debug!("Caller dropped CreateSubscription response receiver");
        }
    }

    /// Looks up the service and sends the request. Returns the call id and service id.
    async fn send_service_call(
        &mut self,
        service_name: &str,
        expected_type: Option<&'static str>, // TODO: likely don't need expected_type here either
        request: &[u8],
    ) -> Result<(CallId, ServiceId), NodeError> {
        let service = self
            .find_service(service_name)
            .ok_or_else(|| NodeError::ServiceNotAdvertised(service_name.to_owned()))?;
        if let Some(expected) = expected_type {
            // if expected service type not equal to type advertised, Err
            if service.r#type != expected {
                return Err(NodeError::ServiceTypeMismatch {
                    expected: expected.to_owned(),
                    advertised: service.r#type.clone(),
                });
            }
        }
        let service_id = service.id;
        let encoding = service
            .request
            .as_ref()
            .map_or(DEFAULT_SERVICE_ENCODING, |request| {
                request.encoding.as_str()
            })
            .to_owned();

        let call_id = self
            .foxglove_client
            .call_service(service_id, &encoding, request)
            .await
            .map_err(NodeError::Foxglove)?;
        debug!(service_name, service_id, call_id, "Service call sent");
        Ok((call_id, service_id))
    }

    fn add_sink(&mut self, id: SubscriptionId, sink: Box<dyn SubscriptionSinkBase>) {
        self.subscriptions.entry(id).or_default().push(sink);
    }

    async fn handle_server_message(&mut self, message: ServerMessage) {
        match message {
            ServerMessage::ServerInfo(server_info) => info!(?server_info),
            ServerMessage::Advertise(advertise) => {
                for channel in advertise.channels {
                    debug!("Advertised channel: {}", channel);

                    // use channel.topic before insert() moves it
                    let opt_pendings = self.pending_subscribes.remove(&channel.topic);
                    let id = channel.id;
                    self.advertised_channels.insert(id, channel);

                    if let Some(mut pendings) = opt_pendings {
                        // remove all pending subscribes who's requester has dropped reply
                        pendings.retain(|pending| !pending.reply.is_closed());

                        // consume entire removed vector
                        for PendingSubscription { sink, reply } in pendings {
                            debug!(
                                "activating subscription for pending subscription on {}",
                                sink.topic_name()
                            );
                            self.activate_subscription(id, sink, reply).await;
                        }
                    }
                }
            }
            ServerMessage::AdvertiseServices(advertise_services) => {
                for service in advertise_services.services {
                    debug!("Advertised service: {}", service);
                    self.advertised_services.insert(service.id, service);
                }
            }
            ServerMessage::UnadvertiseServices(unadvertise_services) => {
                for service_id in unadvertise_services.service_ids {
                    debug!("Unadvertised service: {}", service_id);
                    self.advertised_services.remove(&service_id);
                    self.fail_pending_calls(service_id);
                }
            }
            ServerMessage::ServiceCallResponse(response) => {
                debug!(
                    service_id = response.service_id,
                    call_id = response.call_id,
                    encoding = response.encoding,
                    "Service call response received"
                );
                self.complete_service_call(response.call_id, Ok(response.data));
            }
            ServerMessage::ServiceCallFailure(failure) => {
                warn!(
                    service_id = failure.service_id,
                    call_id = failure.call_id,
                    message = failure.message,
                    "Service call failed"
                );
                self.complete_service_call(
                    failure.call_id,
                    Err(NodeError::ServiceCallFailed(failure.message)),
                );
            }
            ServerMessage::MessageData(message_data) => {
                let Some(sinks) = self.subscriptions.get_mut(&message_data.subscription_id) else {
                    return;
                };

                sinks.retain_mut(|sink| {
                    match sink.send(&message_data.data) {
                        Ok(()) => true,
                        // caller dropped receiver, so delete sink
                        Err(DeliveryError::Closed) => false,
                        Err(DeliveryError::Full) => {
                            // drop this message because queue depth is full
                            warn!(
                                topic = sink.topic_name(),
                                subscription_id = message_data.subscription_id,
                                "Dropped message: subscription queue is full"
                            );
                            true
                        }
                        Err(DeliveryError::Decode(e)) => {
                            error!(
                                topic = sink.topic_name(),
                                ?e,
                                "Failed to decode subscription message"
                            );
                            true
                        }
                    }
                });
            }
        }
    }

    fn find_service(&self, service_name: &str) -> Option<&Service> {
        self.advertised_services
            .values()
            .find(|service| service.name == service_name)
    }

    fn complete_service_call(&mut self, call_id: CallId, result: Result<Bytes, NodeError>) {
        // received response for call_id, remove from pending_calls
        let Some(call) = self.pending_service_calls.remove(&call_id) else {
            debug!(
                call_id,
                "Result for unknown service call (caller may have timed out)"
            );
            return;
        };
        if call.reply.send(result).is_err() {
            debug!(call_id, "Service caller dropped response receiver");
        }
    }

    /// Fails every in-flight call to a service that has gone away
    fn fail_pending_calls(&mut self, service_id: ServiceId) {
        let call_ids: Vec<CallId> = self
            .pending_service_calls
            .iter()
            .filter(|(_, call)| call.service_id == service_id)
            .map(|(call_id, _)| *call_id)
            .collect();
        for call_id in call_ids {
            self.complete_service_call(call_id, Err(NodeError::ServiceUnavailable));
        }
    }
}

#[derive(Clone)]
pub struct NodeHandle {
    node_command_sender: NodeCommandSender,
}

impl NodeHandle {
    /// Creates a subscription whose callback runs on a Tokio task.
    /// Keep the returned handle alive to continue receiving messages.
    pub async fn create_subscription<T>(
        &self,
        topic: &str,
        depth: usize,
        mut callback: impl FnMut(T) + Send + 'static,
    ) -> Result<SubscriptionHandle, NodeError>
    where
        T: RosMessage,
    {
        let (sender, mut receiver) = mpsc::channel::<T>(depth);
        let sink = SubscriptionSink::new(topic.to_owned(), sender);
        let id = self
            .send_request(|reply| NodeCommand::CreateSubscription {
                sink: Box::new(sink),
                reply,
            })
            .await?;

        let task = tokio::spawn(async move {
            while let Some(message) = receiver.recv().await {
                callback(message);
            }
        });

        Ok(SubscriptionHandle::new(id, topic.to_owned(), task))
    }

    pub async fn connect(&self, url: &str) -> Result<(), NodeError> {
        debug!("connect requested");
        self.send_request(|reply| NodeCommand::Connect {
            url: url.to_owned(),
            reply,
        })
        .await
    }

    /// Calls a service and waits up to `timeout` for its response.
    /// Fails with `ServiceTypeMismatch` if the server advertises `service_name` with a type other than `S`.
    pub async fn call_service<S>(
        &self,
        service_name: &str,
        request: &S::Request,
        timeout: Duration,
    ) -> Result<S::Response, NodeError>
    where
        S: RosService,
    {
        let request = request.serialize().map_err(NodeError::Encode)?;
        let response = self
            .call_service_bytes(service_name, Some(S::TYPE_NAME), request, timeout)
            .await?;
        S::Response::deserialize(&response).map_err(NodeError::Decode)
    }

    // TODO: Merge into call_service<S>() because we may not need expected_type to be an Option
    async fn call_service_bytes(
        &self,
        service_name: &str,
        expected_type: Option<&'static str>,
        request: Vec<u8>,
        timeout: Duration,
    ) -> Result<Bytes, NodeError> {
        // Future of byte result from oneshot sender
        let call = self.send_request::<Bytes>(|reply| NodeCommand::CallService {
            service_name: service_name.to_owned(),
            expected_type,
            request,
            reply,
        });
        // On timeout the reply receiver is dropped, the node drops the pending call later
        tokio::time::timeout(timeout, call)
            .await
            .map_err(|_| NodeError::Timeout)?
    }

    /// Generic helper for sending node commands
    async fn send_request<T>(
        &self,
        command_builder: impl FnOnce(oneshot::Sender<Result<T, NodeError>>) -> NodeCommand,
    ) -> Result<T, NodeError> {
        let (reply, response) = oneshot::channel::<Result<T, NodeError>>();
        let command = command_builder(reply);

        self.node_command_sender
            .send(command)
            .await
            .map_err(|_| NodeError::NodeStopped)?; // command coudn't be sent

        response.await.map_err(|_| NodeError::NodeStopped)? // couldn't receive a response from node
    }
}
