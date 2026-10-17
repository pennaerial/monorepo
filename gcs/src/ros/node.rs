use crate::ros::server_types::ServerMessage;
use crate::ros::{
    foxglove_client::FoxgloveClientError, node_command::NodeCommand, FoxgloveClient,
    SubscriptionHandle, SubscriptionId, SubscriptionSinkBase,
};
use dioxus::logger::tracing::{debug, error, info};
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

pub struct RosNode {
    foxglove_client: FoxgloveClient,
    // client allows only one subscription per channelId, so fan out into a Vec of callbacks
    subscriptions: HashMap<SubscriptionId, Vec<Box<dyn SubscriptionSinkBase>>>,
    node_command_receiver: NodeCommandReceiver,
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
            node_command_receiver: receiver,
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
            _ => (),
        }
    }

    async fn handle_server_message(&mut self, message: ServerMessage) {
        info!("{:?}", message);
    }
}

#[derive(Clone)]
pub struct NodeHandle {
    node_command_sender: NodeCommandSender,
}

impl NodeHandle {
    pub fn create_subscription<T: RosMessage>(
        &mut self,
        topic: &str,
        depth: usize,
    ) -> SubscriptionHandle<T> {
        let (_, receiver) = mpsc::channel::<T>(depth);

        SubscriptionHandle::new(0, topic.to_owned(), receiver) // TODO: Make this actually work
    }

    pub async fn connect(&mut self, url: &str) -> Result<(), NodeError> {
        debug!("connect requested");
        self.send_request(|reply| NodeCommand::Connect {
            url: url.to_owned(),
            reply,
        })
        .await
    }

    /// Generic helper for sending node commands
    async fn send_request<T>(
        &mut self,
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
