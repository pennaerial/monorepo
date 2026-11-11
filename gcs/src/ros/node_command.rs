use crate::ros::{node::NodeError, SubscriptionId, SubscriptionSinkBase};
use tokio::sync::oneshot;
use tungstenite::Bytes;

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
    /// `reply` is held until the server answers the call
    CallService {
        service_name: String,
        /// When set, the call fails unless the service is advertised with this type
        expected_type: Option<&'static str>,
        /// CDR-encoded request
        request: Vec<u8>,
        /// CDR-encoded response
        reply: oneshot::Sender<Result<Bytes, NodeError>>,
    },
}
