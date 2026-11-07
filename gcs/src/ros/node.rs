use crate::ros::server_types::{Channel, ChannelId, ServerMessage};
use crate::ros::subscription::DeliveryError;
use crate::ros::SubscriptionSink;
use crate::ros::{
    foxglove_client::FoxgloveClientError, node_command::NodeCommand, FoxgloveClient,
    SubscriptionHandle, SubscriptionId, SubscriptionSinkBase,
};
use dioxus::logger::tracing::{debug, error, info, warn};
use ros_interfaces::RosMessage;
use std::collections::HashMap;
use tokio::sync::{mpsc, oneshot};

pub type NodeCommandReceiver = mpsc::Receiver<NodeCommand>;
pub type NodeCommandSender = mpsc::Sender<NodeCommand>;

#[derive(Debug)]
pub enum NodeError {
    NodeStopped,
    Foxglove(FoxgloveClientError),
    // Add node-specific failures as they become necessary.
}

/// stores all necessary data for a pending subscription
struct PendingSubscription {
    sink: Box<dyn SubscriptionSinkBase>,
    reply: oneshot::Sender<Result<SubscriptionId, NodeError>>,
}

pub struct RosNode {
    foxglove_client: FoxgloveClient,
    /// client allows only one subscription per channelId, so fan out into a Vec of callbacks
    subscriptions: HashMap<SubscriptionId, Vec<Box<dyn SubscriptionSinkBase>>>,
    /// for subscription on topics that haven't been advertised yet
    pending_subscribes: HashMap<String, Vec<PendingSubscription>>,
    /// for receiving NodeCommands sent from a NodeHandle
    node_command_receiver: NodeCommandReceiver,
    /// Tracked advertised channels from FoxgloveClient
    advertised_channels: HashMap<ChannelId, Channel>,
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
                                "activating subscription for pending publish on {}",
                                sink.topic_name()
                            );
                            self.activate_subscription(id, sink, reply).await;
                        }
                    }
                }
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
            _ => (),
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
