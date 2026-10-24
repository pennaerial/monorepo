use crate::components::{Button, SideBar};
use crate::hooks::use_ros_node;
use crate::ros::server_types::ServerMessage;
use crate::ros::{FoxgloveClient, SubscriptionHandle};
use dioxus::prelude::*;
use ros_interfaces::{geometry_msgs, std_msgs};

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
    let node = use_ros_node();
    let mut string_msg = use_signal_sync(String::new);

    // use a signal here keep SubscriptionHandle alive
    let mut subscription = use_signal(|| None::<SubscriptionHandle>);

    rsx! {

        div { class: "app-layout",
            SideBar {}

            main { class: "main-content",

                h1 { class: "page-title", "Mission Control Page" }

                Button {
                    label: "Connect",
                    onclick: move |_| {
                        let node = node.read().clone();
                        async move {
                            if let Err(e) = node.connect("ws://localhost:8765").await {
                                info!("Failed: {:?},", e);
                            } else {
                                info!("Successful connect")
                            }
                        }
                    },
                }
            }
        }

        Button {
            label: "Subscribe",
            onclick: move |_| {
                let node = node.read().clone();

                async move {
                    let result = node
                        .create_subscription::<
                            std_msgs::msg::String,
                            _,
                        >("/chatter", 10, move |message| string_msg.set(message.data))
                        .await;
                    match result {
                        Ok(handle) => subscription.set(Some(handle)),
                        Err(error) => error!(? error, "Failed to subscribe to /chatter"),
                    }
                }
            },
        }
        p { "{string_msg}" }
    }
}
