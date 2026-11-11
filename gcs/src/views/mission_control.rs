use crate::components::{Button, LogPanel, SideBar};
use crate::hooks::{use_ros_node, use_subscription};
use dioxus::prelude::*;
use ros_interfaces::{example_interfaces, std_msgs};
use std::time::Duration;

const SERVICE_TIMEOUT: Duration = Duration::from_secs(5);

#[component]
pub fn MissionControl() -> Element {
    let node = use_ros_node();
    let mut string_msg = use_signal_sync(String::new);
    let chatter_sub =
        use_subscription::<std_msgs::msg::String>(node.read().clone(), "/chatter", 10);
    let mut service_result = use_signal(String::new);

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
                Button {
                    label: "Subscribe",
                    onclick: move |_| {
                        let mut sub = chatter_sub.clone();
                        async move {
                            if let Err(e) = sub.subscribe(move |msg| { string_msg.set(msg.data) }).await
                            {
                                error!("failed to subscribe to chatter!: {:?}", e);
                            }
                        }
                    },
                }
                p { "{string_msg}" }

                // TODO: Think about creating some ServiceClient abstraction on top such that you
                // can repeat service calls to an initiailzed service, similar to create_client in ROS
                // Service call demos
                Button {
                    label: "Add 2 + 3",
                    onclick: move |_| {
                        let node = node.read().clone();
                        async move {
                            let request = example_interfaces::srv::AddTwoInts_Request { a: 2, b: 3 };
                            let result = node
                                .call_service::<example_interfaces::srv::AddTwoInts>(
                                    "/add_two_ints",
                                    &request,
                                    SERVICE_TIMEOUT,
                                )
                                .await;
                            service_result
                                .set(
                                    match result {
                                        Ok(response) => format!("/add_two_ints: 2 + 3 = {}", response.sum),
                                        Err(e) => format!("/add_two_ints: {e}"),
                                    },
                                );
                        }
                    },
                }
                p { "{service_result}" }
                LogPanel {}

            }
        }

    }
}
