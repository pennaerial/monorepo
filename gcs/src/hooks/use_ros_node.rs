use dioxus::prelude::*;

use crate::ros::NodeHandle;

/// Stores the shared node handle in a copyable signal for UI callbacks.
pub fn use_ros_node() -> Signal<NodeHandle> {
    let node = use_context::<NodeHandle>();
    use_signal(move || node)
}
