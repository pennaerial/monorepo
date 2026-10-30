use ros_interfaces::RosMessage;
use tokio::sync::mpsc;

pub type SubscriptionId = u32;

#[derive(Debug)]
pub enum DeliveryError {
    Decode(cdr::Error),
    Full,
    Closed,
}

// allows RosNode to dynamically store SubscriptionBase trait objects
pub trait SubscriptionSinkBase: Send {
    fn topic_name(&self) -> &str;
    fn is_closed(&self) -> bool;
    fn send(&mut self, bytes: &[u8]) -> Result<(), DeliveryError>;
}

/// Type for sending data to a Subscription<T>'s receiver pair
pub struct SubscriptionSink<T> {
    topic_name: String,
    sender: mpsc::Sender<T>,
}

impl<T: RosMessage> SubscriptionSink<T> {
    pub fn new(topic_name: String, sender: mpsc::Sender<T>) -> Self {
        Self { topic_name, sender }
    }
}

impl<T: RosMessage> SubscriptionSinkBase for SubscriptionSink<T> {
    fn topic_name(&self) -> &str {
        &self.topic_name
    }

    fn is_closed(&self) -> bool {
        self.sender.is_closed()
    }

    fn send(&mut self, bytes: &[u8]) -> Result<(), DeliveryError> {
        let message = T::deserialize(bytes).map_err(DeliveryError::Decode)?;
        self.sender.try_send(message).map_err(|error| match error {
            tokio::sync::mpsc::error::TrySendError::Full(_) => DeliveryError::Full,
            tokio::sync::mpsc::error::TrySendError::Closed(_) => DeliveryError::Closed,
        })?;
        Ok(())
    }
}

// Caller side

/// Keeps a subscription callback running until this handle is dropped.
pub struct SubscriptionHandle {
    subscription_id: SubscriptionId,
    topic_name: String,
    task: tokio::task::JoinHandle<()>,
}

impl SubscriptionHandle {
    pub fn new(
        subscription_id: SubscriptionId,
        topic_name: String,
        task: tokio::task::JoinHandle<()>,
    ) -> Self {
        Self {
            subscription_id,
            topic_name,
            task,
        }
    }
}

impl Drop for SubscriptionHandle {
    fn drop(&mut self) {
        self.task.abort();
    }
}
