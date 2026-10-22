use crate::components::{Button, SideBar};
use crate::ros::server_types::ServerMessage;
use crate::ros::{FoxgloveClient, NodeHandle};
use dioxus::prelude::*;
use ros_interfaces::geometry_msgs;

// / * `url`: e.g. "localhost:8765"
async fn run_client(url: &str) {
    let mut client = FoxgloveClient::new();

    info!(%url, "Connecting to Foxglove bridge");
    if let Err(error) = client.connect(url).await {
        error!(%url, error = %error, "Foxglove connection failed");
        return;
    }

    info!(%url, "Connected to Foxglove bridge");

    // testing a hardcoded 4 id
    match client.subscribe(4).await {
        Ok(id) => info!(subscription_id = id, "Subscribed to channel 4"),
        Err(err) => error!(error = %err, "Subscription failed"),
    }

    let mut receiver = client.get_receiver();
    tokio::spawn(async move {
        loop {
            let msg = receiver.recv().await;
            match msg {
                Ok(ServerMessage::MessageData(data)) => {
                    match cdr::deserialize::<geometry_msgs::msg::Pose>(data.data.as_ref()) {
                        Ok(decoded) => trace!(?decoded, "Received pose"),
                        Err(error) => warn!(%error, "Failed to deserialize pose"),
                    }
                }
                Ok(ServerMessage::Advertise(adv)) => {
                    debug!(?adv, "Received channel advertisement")
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    debug!("Message channel closed; stopping receiver");
                    break;
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(count)) => {
                    warn!(count, "Receiver missed messages");
                }
                _ => (),
            }
        }
    });
}

#[component]
pub fn MissionControl() -> Element {
    let node = use_context::<NodeHandle>();

    rsx! {

        div {
            class: "app-layout",
            SideBar {}

            main {
                class: "main-content",

                h1 { class: "page-title",
                "Mission Control Page" 
                }


                Button {
                    label: "Connect",
                    onclick: move |_| {
                        let mut node = node.clone();
                        async move {
                            if let Err(e) = node.connect("ws://localhost:8765").await {
                                info!("Failed: {:?},", e);
                            } else {
                                info!("Successful connect")
                            }
                        }
                    }
                }        
            }
        }
    }
}
        

