mod components;
mod hooks;
use gcs::ros::{self, NodeHandle};
mod routes;
mod views;

use dioxus::prelude::*;
use dioxus::{logger, logger::tracing};

use ros_interfaces::rcl_interfaces::msg::Log;
use routes::Route;

use crate::hooks::use_subscription;
use crate::ros::RosNode;

const TAILWIND_CSS: Asset = asset!("/assets/tailwind.css");

fn main() {
    logger::init(tracing::Level::DEBUG).expect("Failed to initialize logger");
    info!("Starting PennAiR ground control station");
    dioxus::launch(App);
}

#[component]
fn App() -> Element {
    use_context_provider(|| components::SidebarContext {
        expanded: Signal::new(false), // sidebar is closed by default at launch
    });
    let node_handle = use_context_provider(|| {
        let (node_handle, mut node) = RosNode::new(64); // channel depth 64

        tokio::spawn(async move {
            if let Err(e) = node.spin().await {
                error!("Node stopped: {:?}", e);
            }
        });
        node_handle
    });

    let rosout_sub = use_subscription::<Log>(node_handle, "/rosout", 10);
    let mut logs = use_signal_sync(components::LogHistory::new);
    use_context_provider(move || logs);

    use_future(move || {
        let mut sub = rosout_sub.clone();
        async move {
            let res = sub
                .subscribe(move |msg| {
                    logs.write().add_log(msg);
                })
                .await;

            if let Err(error) = res {
                error!("Failed to subscribe to /rosout: {error:?}");
            }
        }
    });

    rsx! {
        document::Link { rel: "stylesheet", href: TAILWIND_CSS }
        Router::<Route> {}
    }
}
