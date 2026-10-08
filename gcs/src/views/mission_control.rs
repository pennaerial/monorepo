use tokio::time::{sleep, Duration};

use crate::components::Button;
use crate::ros::server_types::ServerMessage;
use crate::ros::FoxgloveClient;
use dioxus::prelude::*;
use ros_interfaces::{geometry_msgs, std_msgs};

// / * `url`: e.g. "localhost:8765"
async fn run_client(url: &str) {
    let mut client = FoxgloveClient::new();
    let string = std_msgs::msg::String {
        data: String::from("Hello"),
    };
    println!("std_msgs::msg::String: {}", string.data);
    match client.connect(url).await {
        Ok(()) => (),
        Err(error) => eprintln!("connection failed! {error}"),
    }

    // testing a hardcoded 4 id
    match client.subscribe(4).await {
        Ok(id) => println!("SUCCESSFUL channel sub: {id}"),
        Err(err) => println!("SUBCRIBE ERROR: {err}"),
    }

    match client.get_receiver() {
        Ok(mut receiver) => {
            tokio::spawn(async move {
                loop {
                    let msg = receiver.recv().await;
                    match msg {
                        Ok(ServerMessage::MessageData(data)) => {
                            match cdr::deserialize::<geometry_msgs::msg::Pose>(data.data.as_ref()) {
                                Ok(decoded) => println!("successful cdr: {:?}", decoded),
                                Err(e) => println!("Failed cdr deserialization, {e}"),
                            }
                        }
                        Err(e) => println!("{e}"),
                        _ => (),
                    }
                    sleep(Duration::from_secs(1)).await;
                    println!("receiver loop");
                }
            });
        }
        Err(e) => println!("{e}"),
    }
}

#[component]
pub fn MissionControl() -> Element {
    rsx! {
        h1 { "Mission Control Page" }
        Button {
            label: "Connect",
            onclick:  |_| async move {
                run_client("ws://localhost:8765").await
            }
        }
    }
}
