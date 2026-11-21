use crate::ros::{node::NodeError, SubscriptionId, SubscriptionSinkBase};
use bytes::Bytes;
use tokio::sync::oneshot;

pub enum NodeCommand {
    Connect {
        url: String,
        reply: oneshot::Sender<Result<(), NodeError>>,
    },
    // TODO: Plz implement this
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
        expected_type: Option<String>,
        /// Request encoded using the service's advertised request encoding
        request: Vec<u8>,
        /// Response bytes using the service's advertised response encoding
        reply: oneshot::Sender<Result<Bytes, NodeError>>,
    },
}
