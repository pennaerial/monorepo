use ros_interfaces::RosMessage;

pub type SubscriptionCallback<T> = Box<dyn FnMut(T)>;
pub type SubscriptionId = u32;

// allows RosNode to dynamically point to any subscription that implements subscription base
pub trait SubscriptionBase {
    fn topic_name(&self) -> &str;

    // run the callback for a SubscriptionBase interface
    fn handle_callback(&mut self, bytes: &[u8]) -> Result<(), cdr::Error>;
}

pub struct Subscription<T> {
    topic_name: String,
    // subscription_id: SubscriptionId, // does this need to be here?
    callback: SubscriptionCallback<T>,
}

impl<T> Subscription<T> {
    pub fn new(topic_name: String, callback: SubscriptionCallback<T>) -> Self {
        Self {
            topic_name,
            callback,
        }
    }
}

impl<T: RosMessage> SubscriptionBase for Subscription<T> {
    fn topic_name(&self) -> &str {
        &self.topic_name
    }

    fn handle_callback(&mut self, bytes: &[u8]) -> Result<(), cdr::Error> {
        let message = T::deserialize(bytes)?;
        (self.callback)(message);
        Ok(())
    }

}
//
// type TopicName = String;
