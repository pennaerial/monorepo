use crate::components::{Button, LogPanel, SideBar};
use crate::hooks::{use_ros_node, use_subscription};
use dioxus::prelude::*;
use gcs::deployment::{BrowserDeployment, DeploymentClient, DeploymentProgress};
use ros_interfaces::{example_interfaces, std_msgs, std_srvs};
use std::time::Duration;

const SERVICE_TIMEOUT: Duration = Duration::from_secs(5);
const DRONE_ONE: &'static str = "uav_0";

#[component]
pub fn MissionControl() -> Element {
    let node = use_ros_node();
    let mut server_url = use_signal(|| "ws://localhost:8765".to_owned());
    let mut deployment_token = use_signal(String::new);
    let selected_deployment = use_signal(|| None::<BrowserDeployment>);
    let deployment_status = use_signal(|| "Select the repository folder to deploy".to_owned());
    let mut string_msg = use_signal_sync(String::new);
    let chatter_sub =
        use_subscription::<std_msgs::msg::String>(node.read().clone(), "/chatter", 10);
    let mut service_result = use_signal(String::new);
    let mut trigger_result = use_signal(String::new);

    rsx! {

        div { class: "app-layout",
            SideBar {}

            main { class: "main-content",

                h1 { class: "page-title", "Mission Control Page" }

                section {class: "rounded-lg border border-slate-200 bg-whitebg p-4",

                    h2 {class: "mb-3 font-semibold", "Connecting and Subscribing"}

                    div {class: "flex flex-wrap items-center gap-3",

                        input {
                            class: "rounded-md border border-slate-300 px-3 py-2 text-sm",
                            r#type: "url",
                            value: "{server_url}",
                            placeholder: "ws://air-04.local:8765",
                            oninput: move |event| server_url.set(event.value()),
                        }

                        Button {
                            label: "Connect",
                            onclick: move |_| {
                                let node = node.read().clone();
                                let url = server_url.read().clone();
                                let mut status = deployment_status;
                                async move {
                                    if let Err(e) = node.connect(&url).await {
                                        info!("Failed: {:?},", e);
                                        status.set(format!("Connection failed: {e}"));
                                    } else {
                                        info!("Successful connect");
                                        status.set("Connected to Foxglove server".into());
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
                    }

                    if !string_msg.read().is_empty() {
                        p { class: "mb-3 break-words text-sm text-slate-600",
                        "{string_msg}" }
                    }

                }

                section {class: "rounded-lg border border-slate-200 bg-whitebg p-4",

                    h2 {class: "mb-3 font-semibold", "Deploy to Raspberry Pi"}

                    div {class: "flex flex-wrap items-center gap-3",
                        input {
                            class: "rounded-md border border-slate-300 px-3 py-2 text-sm",
                            r#type: "password",
                            value: "{deployment_token}",
                            placeholder: "Deployment token",
                            oninput: move |event| deployment_token.set(event.value()),
                        }

                        input {
                            class: "rounded-md border border-slate-300 px-3 py-2 text-sm",
                            r#type: "file",
                            directory: true,
                            multiple: true,
                            onchange: move |event| {
                                let files = event.files();
                                let mut deployment = selected_deployment;
                                let mut status = deployment_status;
                                async move {
                                    status.set(format!("Reading and hashing {} selected files", files.len()));
                                    match BrowserDeployment::from_selected_folder(files, true).await {
                                        Ok(selected) => {
                                            let count = selected.manifest.files.len();
                                            deployment.set(Some(selected));
                                            status.set(format!("Ready to deploy {count} files"));
                                        }
                                        Err(error) => {
                                            deployment.set(None);
                                            status.set(format!("Folder selection failed: {error}"));
                                        }
                                    }
                                }
                            },
                        }

                        Button {
                            label: "Deploy selected repository",
                            onclick: move |_| {
                                let node = node.read().clone();
                                let token = deployment_token.read().clone();
                                let mut deployment_signal = selected_deployment;
                                let mut status = deployment_status;
                                async move {
                                    let Some(mut deployment) = deployment_signal.write().take() else {
                                        status.set("Select the repository folder before deploying".into());
                                        return;
                                    };
                                    status.set("Authenticating deployment session".into());
                                    let result = match DeploymentClient::authenticate(node, token).await {
                                        Ok(mut client) => client
                                            .deploy(&mut deployment, |progress| {
                                                status.set(progress_message(progress));
                                            })
                                            .await,
                                        Err(error) => Err(error),
                                    };
                                    deployment_signal.set(Some(deployment));
                                    if let Err(error) = result {
                                        status.set(format!("Deployment failed: {error}"));
                                    }
                                }
                            },
                        }
                    }

                    p { class: "mt-3 break-words text-sm text-slate-600", "{deployment_status}" }
                }



                // TODO: Think about creating some ServiceClient abstraction on top such that you
                // can repeat service calls to an initiailzed service, similar to create_client in ROS
                // Service call demos


                section {class: "rounded-lg border border-slate-200 bg-whitebg p-4",

                    h2 {class: "mb-3 font-semibold", "Service Call"}

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

                    if !service_result.read().is_empty() {
                        p { class: "mb-3 break-words text-sm text-slate-600",
                        "{service_result}" }
                    }
                }

                section {class: "rounded-lg border border-slate-200 bg-whitebg p-4",

                    h2 {class: "mb-3 font-semibold", "Launching"}

                    Button {
                        label: "Launch (requires auto-launch off)",
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

fn progress_message(progress: DeploymentProgress) -> String {
    match progress {
        DeploymentProgress::Prepared { files, required } => {
            format!("Prepared {files} files; {required} require transfer")
        }
        DeploymentProgress::FileStaged {
            path,
            completed,
            total,
        } => format!("Staged {path} ({completed}/{total})"),
        DeploymentProgress::Activated { files, deletions } => {
            format!("Activated {files} files and applied {deletions} deletions")
        }
        DeploymentProgress::BuildOutput(line) => line.line,
        DeploymentProgress::Restarted { service } => format!("Restarted {service}"),
        DeploymentProgress::Complete { deployment_id } => {
            format!("Deployment {deployment_id} completed")
        }
    }
}
