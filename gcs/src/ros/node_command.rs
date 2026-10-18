use crate::ros::{node::NodeError, SubscriptionId, SubscriptionSinkBase};
use tokio::sync::oneshot;

pub enum NodeCommand {
    Connect {
        url: String,
        reply: oneshot::Sender<Result<(), NodeError>>,
    },
    Disconnect {
        reply: oneshot::Sender<Result<(), NodeError>>,
    },
    CreateSubscription {
        sink: Box<dyn SubscriptionSinkBase>,
        reply: oneshot::Sender<Result<SubscriptionId, NodeError>>,
    },
    // DestroySubscription,
}
