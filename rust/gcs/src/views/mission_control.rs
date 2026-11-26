use crate::components::{Button, LogPanel, SideBar};
use crate::ros::NodeHandle;
use dioxus::prelude::*;
use futures_util::future::join_all;
use ros_interfaces::std_srvs;
use std::time::Duration;

const SERVICE_TIMEOUT: Duration = Duration::from_secs(5);
const DRONE_ONE: &'static str = "uav_0";

// For single vs fleet vehicle launches
#[derive(Clone, Copy, PartialEq)]
enum MissionControlMode {
    Standalone,
    Fleet,
}

impl MissionControlMode {
    fn label(&self) -> &'static str {
        match self {
            Self::Standalone => "Standalone",
            Self::Fleet => "Fleet",
        }
    }
}

#[component]
pub fn MissionControl() -> Element {
    let node = use_context::<NodeHandle>();
    let mut control_mode = use_signal(|| MissionControlMode::Standalone);

    rsx! {
        div { class: "app-layout",
            SideBar {}

            main { class: "min-w-0 flex-1 box-border p-2",

                // Mission control header
                section { class: "flex items-center justify-between",
                    h1 { "Mission Control" }

                    label { r#for: "mission-control-mode", "Mode: {control_mode().label()}" }
                    select {
                        id: "mission-control-mode",
                        oninput: move |event| match event.value().as_str() {
                            "standalone" => control_mode.set(MissionControlMode::Standalone),
                            "fleet" => control_mode.set(MissionControlMode::Fleet),
                            _ => ()
                        },
                        option { value: "standalone", "Standalone" }
                        option { value: "fleet", "Fleet" }
                    }

                    Button {
                        label: "Connect Foxglove Bridge",
                        onclick: move |_| {
                            let node = node.clone();
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

                // change view based on standalone vs fleet
                {match control_mode() {
                    MissionControlMode::Standalone => rsx! { StandaloneControl {} },
                    MissionControlMode::Fleet => rsx! { FleetControl {} },
                }}
            }
        }
    }
}

#[component]
fn StandaloneControl() -> Element {
    let node = use_context::<NodeHandle>();
    let mut trigger_result = use_signal(String::new);

    rsx! {

        // TODO: Think about creating some ServiceClient abstraction on top such that you
        // can repeat service calls to an initiailzed service, similar to create_client in ROS
        section { class: "rounded-lg border border-slate-200 bg-whitebg p-4",
            h2 { class: "mb-3 font-semibold", "Launching" }

            Button {
                label: "Start Mission Trigger",
                onclick: move |_| {
                    let node = node.clone();
                    async move {
                        let request = std_srvs::srv::Trigger_Request { structure_needs_at_least_one_member: 0 };
                        let result = node
                            .call_service::<std_srvs::srv::Trigger>(
                                &format!("/{DRONE_ONE}/mode_manager/start_mission"),
                                &request,
                                SERVICE_TIMEOUT,
                            )
                            .await;
                        trigger_result.set(match result {
                            Ok(response) => format!("/{DRONE_ONE}/mode_manager/start_mission: launch success = {}", response.success),
                            Err(e) => format!("/{DRONE_ONE}/mode_manager/start_mission: {e}"),
                        });
                    }
                },
            }

            if !trigger_result.read().is_empty() {
                p { class: "mb-3 break-words text-sm text-slate-600",
                    "{trigger_result}"
                }
            }
        }

        section { class: "mt-8",
            h2 { class: "mb-3 text-lg font-semibold", "Log Panel" }

            LogPanel {}
        }
    }
}

#[component]
fn FleetControl() -> Element {
    let node = use_context::<NodeHandle>();
    let mut trigger_result = use_signal(String::new);

    let uav_ids: &[&str] = &["uav_0", "uav_1"];

    rsx! {
        section { class: "rounded-lg border border-slate-200 bg-whitebg p-4",
            h2 { class: "mb-3 font-semibold", "Launch" }

            Button {
                label: "Launch fleet (uav_0 and uav_1)",
                onclick: move |_| {
                    let node = node.clone();
                    async move {
                        let node = &node;
                        let calls: Vec<_> = uav_ids
                            .iter()
                            .copied()
                            .map(|id| async move {
                                let service = format!("/{id}/mode_manager/start_mission");
                                let request = std_srvs::srv::Trigger_Request {
                                    structure_needs_at_least_one_member: 0,
                                };

                                let result = node
                                    .call_service::<std_srvs::srv::Trigger>(
                                        &service,
                                        &request,
                                        SERVICE_TIMEOUT,
                                    )
                                    .await;

                                match result {
                                    Ok(response) => format!("{id}: launch success = {}", response.success),
                                    Err(error) => format!("{id}: {error}"),
                                }
                            })
                            .collect();

                        // wait for all services to finish concurrently
                        let results = futures_util::future::join_all(calls).await;
                        trigger_result.set(results.join("\n"));
                    }
                },
            }

            if !trigger_result.read().is_empty() {
                p { class: "whitespace-pre-line", "{trigger_result}" }
            }
        }


        section { class: "mt-8",
            h2 { class: "mb-3 text-lg font-semibold", "Log Panel" }
            LogPanel {}
        }
    }
}
