use crate::components::{Button, LogPanel, SideBar};
use crate::hooks::{use_ros_node, use_subscription};
use dioxus::prelude::*;
use ros_interfaces::std_srvs;
use std::time::Duration;

const SERVICE_TIMEOUT: Duration = Duration::from_secs(5);
const DRONE_ONE: &'static str = "uav_0";

#[component]
pub fn MissionControl() -> Element {
    let node = use_ros_node();
    let mut trigger_result = use_signal(String::new);

    rsx! {

        div { class: "app-layout",
            SideBar {}

            main { class: "main-content",

                h1 { class: "page-title", "Mission Control" }

                section {class: "rounded-lg border border-slate-200 bg-whitebg p-4",

                    h2 {class: "mb-3 font-semibold", "Connect"}

                    div {class: "flex flex-wrap items-center gap-3",

                        Button {
                            label: "Connect Foxglove Bridge",
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

                // TODO: Think about creating some ServiceClient abstraction on top such that you
                // can repeat service calls to an initiailzed service, similar to create_client in ROS
                section {class: "rounded-lg border border-slate-200 bg-whitebg p-4",

                    h2 {class: "mb-3 font-semibold", "Launching"}

                    Button {
                        label: "Start Mission Trigger",
                        onclick: move |_| {
                            let node = node.read().clone();
                            async move {
                                let request = std_srvs::srv::Trigger_Request { structure_needs_at_least_one_member: 0 };
                                let result = node
                                    .call_service::<std_srvs::srv::Trigger>(
                                        &format!("/{DRONE_ONE}/mode_manager/start_mission"),
                                        &request,
                                        SERVICE_TIMEOUT,
                                    )
                                    .await;
                                trigger_result
                                    .set(
                                        match result {
                                            Ok(response) => format!("/{DRONE_ONE}/mode_manager/start_mission: launch success = {}", response.success),
                                            Err(e) => format!("/{DRONE_ONE}/mode_manager/start_mission: {e}"),
                                        },
                                    );
                            }
                        },
                    }

                    if !trigger_result.read().is_empty() {
                        p { class: "mb-3 break-words text-sm text-slate-600",
                        "{trigger_result}" }
                    }

                }

                section { class: "mt-8",
                    h2{class: "mb-3 text-lg font-semibold", "Log Panel"}
                    LogPanel {}
                }
            }
        }

    }
}
