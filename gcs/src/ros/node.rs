// crate::ros::subscription::SubscriptionId!

use std::collections::HashMap;
use ros_interfaces::RosMessage;
use crate::ros::{FoxgloveClient, Subscription, SubscriptionBase, SubscriptionId, foxglove_client::{FoxgloveClientError}};
use crate::ros::server_types::ServerMessage;


pub struct RosNode {
    foxglove_client: FoxgloveClient,
    // receiver: ServerMessageReceiver,
    // client allows only one subscription per channelId, so fan out into a Vec of callbacks
    subscriptions: HashMap<SubscriptionId, Vec<Box<dyn SubscriptionBase>>>,
}

impl RosNode {
    // foxglove_client must already be successfully connected
    pub fn new(foxglove_client: FoxgloveClient) -> Result<Self, FoxgloveClientError> {
        if !foxglove_client.is_connected() {
            return Err(FoxgloveClientError::NotConnected);
        }

        Ok(Self {
            foxglove_client,
            subscriptions: HashMap::new()
        })
    }

    /// Runs the node's receive loop and callbacks
    pub async fn spin(&mut self) -> Result<(), FoxgloveClientError> {
        let mut receiver = self.foxglove_client.get_receiver();
        loop {
            let message = receiver.recv().await;
            match message {
                Ok(ServerMessage::Advertise(advertise)) => {},
                _ => {},
            }
        }
    }


    pub fn create_subscription<T: RosMessage + 'static>(&mut self, topic: &str, callback: impl FnMut(T) + 'static) {
        let subscription: Subscription<T> = Subscription::new(topic.to_owned(), Box::new(callback));
        // self.foxglove_client.subscribe(channel_id)

        // 0 is hardcoded value for now, until we create a <String, Id> mapping
        self.subscriptions.insert(0, vec![Box::new(subscription)]);
    }
}
