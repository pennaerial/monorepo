use dioxus::prelude::*;
use crate::components::Button;
use crate::ros::FoxgloveClient;
use ros_interfaces::std_msgs;



/// * `url`: e.g. "localhost:8765"
async fn run_client(url: &str) {
    let mut client = FoxgloveClient::new();
    let string = std_msgs::msg::String { data: String::from("Hello") };
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
