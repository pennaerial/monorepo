use dioxus::prelude::*;
use ros_interfaces::RosMessage;
use std::marker::PhantomData;

use crate::ros::{NodeError, NodeHandle, SubscriptionHandle};

/// Stores the shared node handle in a copyable signal for UI callbacks.
pub fn use_ros_node() -> Signal<NodeHandle> {
    let node = use_context::<NodeHandle>();
    use_signal(move || node)
}

#[derive(Clone)]
pub struct UseSubscription<T> {
    node: NodeHandle,
    topic: String,
    depth: usize,
    signal: Signal<Option<SubscriptionHandle>>,
    /// associate with a type T without storing additional space
    _message: PhantomData<fn(T)>,
}

impl<T: RosMessage> UseSubscription<T> {
    pub async fn subscribe(
        &mut self,
        callback: impl FnMut(T) + Send + 'static,
    ) -> Result<(), NodeError> {
        let result = self
            .node
            .create_subscription::<T>(&self.topic, self.depth, callback)
            .await?;
        self.signal.set(Some(result));
        Ok(())
    }
}

pub fn use_subscription<T: RosMessage>(
    node: NodeHandle,
    topic: &str,
    depth: usize,
) -> UseSubscription<T> {
    UseSubscription {
        node,
        topic: topic.to_owned(),
        depth,
        signal: use_signal(|| None::<SubscriptionHandle>),
        _message: PhantomData,
    }
}
