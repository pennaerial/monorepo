mod components;
mod hooks;
mod ros;
mod routes;
mod views;

use dioxus::prelude::*;
use dioxus::{logger, logger::tracing};

use routes::Route;

use crate::ros::RosNode;

const TAILWIND_CSS: Asset = asset!("/assets/tailwind.css");

fn main() {
    logger::init(tracing::Level::DEBUG).expect("Failed to initialize logger");
    info!("Starting PennAiR ground control station");
    dioxus::launch(App);
}

#[component]
fn App() -> Element {
    use_context_provider(|| {
        let (node_handle, mut node) = RosNode::new(64); // channel depth 64

        tokio::spawn(async move {
            if let Err(e) = node.spin().await {
                error!("Node stopped: {:?}", e);
            }
        });
        node_handle
    });

    rsx! {
        document::Link { rel: "stylesheet", href: TAILWIND_CSS }
        Router::<Route> {}
    }
}
