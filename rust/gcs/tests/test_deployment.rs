use foxglove::{ChannelBuilder, Context, Schema};
use gcs::{
    deployment::{BrowserDeployment, DeploymentClient, DeploymentProgress},
    ros::RosNode,
};
use pennair_agent::{config::AgentConfig, server::AgentServer};
use ros_interfaces::{std_msgs, RosMessage};
use std::{fs, net::SocketAddr, path::PathBuf, sync::Arc, time::Duration};
use tokio::sync::mpsc;
use tokio::time::timeout;

const TOKEN: &str = "foxglove-deployment-test-token-0123456789";

fn unique_test_root() -> PathBuf {
    std::env::temp_dir().join(format!(
        "pennair-foxglove-deployment-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ))
}

#[tokio::test]
async fn gcs_deploys_through_foxglove_services() {
    let root = unique_test_root();
    let config = AgentConfig {
        bind_address: "127.0.0.1:0".parse::<SocketAddr>().unwrap(),
        agent_name: "air-test".into(),
        token: TOKEN.into(),
        deployment_root: root.clone(),
        build_shell: PathBuf::from("/bin/bash"),
        ros_setup: PathBuf::from("/dev/null"),
        colcon: PathBuf::from("/usr/bin/true"),
        sudo: PathBuf::from("/usr/bin/true"),
        systemctl: PathBuf::from("/usr/bin/true"),
        service_name: "pennair-autonomy.service".into(),
    };
    let context = Arc::new(Context::new());
    let telemetry = ChannelBuilder::new("/deployment_test")
        .context(&context)
        .message_encoding("cdr")
        .schema(Schema::new(
            std_msgs::msg::String::TYPE_NAME,
            "ros2msg",
            &[][..],
        ))
        .build_raw()
        .unwrap();
    let (server, _, _) = AgentServer::new(config)
        .with_context(Arc::clone(&context))
        .start_async()
        .await
        .unwrap();
    let url = format!("ws://127.0.0.1:{}", server.port());

    let (node_handle, mut node) = RosNode::new(64);
    let node_task = tokio::spawn(async move { node.spin().await });
    node_handle.connect(&url).await.unwrap();

    let (telemetry_sender, mut telemetry_receiver) = mpsc::unbounded_channel();
    let telemetry_subscription = node_handle
        .create_subscription::<std_msgs::msg::String>("/deployment_test", 10, move |message| {
            let _ = telemetry_sender.send(message);
        })
        .await
        .unwrap();
    while !telemetry.has_sinks() {
        tokio::task::yield_now().await;
    }
    let expected_telemetry = std_msgs::msg::String {
        data: "same Foxglove connection".into(),
    };
    telemetry.log(&expected_telemetry.serialize().unwrap());
    assert_eq!(
        timeout(Duration::from_secs(2), telemetry_receiver.recv())
            .await
            .expect("telemetry timed out")
            .expect("telemetry channel closed"),
        expected_telemetry
    );

    let monorepo_config = br#"[deploy]
paths = ["monorepo.toml", "ros_ws/src/package"]
"#;
    let mut deployment = BrowserDeployment::from_named_files(
        vec![
            ("repo/monorepo.toml".into(), monorepo_config.to_vec()),
            ("repo/ros_ws/src/package/a.txt".into(), b"alpha".to_vec()),
            ("repo/ros_ws/src/package/b.txt".into(), b"beta".to_vec()),
        ],
        true,
    )
    .unwrap();
    let deployment_id = deployment.manifest.deployment_id.clone();

    let mut client = DeploymentClient::authenticate(node_handle.clone(), TOKEN.into())
        .await
        .unwrap();
    let (agent_name, _) = client.health().await.unwrap();
    assert_eq!(agent_name, "air-test");

    let mut first_progress = Vec::new();
    client
        .deploy(&mut deployment, |progress| first_progress.push(progress))
        .await
        .unwrap();
    assert!(matches!(
        first_progress.first(),
        Some(DeploymentProgress::Prepared {
            files: 3,
            required: 3
        })
    ));
    assert!(matches!(
        first_progress.last(),
        Some(DeploymentProgress::Complete { deployment_id: id }) if id == &deployment_id
    ));
    assert_eq!(
        fs::read(root.join("ros_ws/src/package/a.txt")).unwrap(),
        b"alpha"
    );
    assert_eq!(
        fs::read(root.join("ros_ws/src/package/b.txt")).unwrap(),
        b"beta"
    );
    assert_eq!(
        fs::read_to_string(root.join(".pennair/built"))
            .unwrap()
            .trim(),
        deployment_id
    );

    let mut repeat_progress = Vec::new();
    client
        .deploy(&mut deployment, |progress| repeat_progress.push(progress))
        .await
        .unwrap();
    assert!(matches!(
        repeat_progress.first(),
        Some(DeploymentProgress::Prepared { required: 0, .. })
    ));

    drop(telemetry_subscription);
    drop(client);
    drop(node_handle);
    timeout(Duration::from_secs(2), node_task)
        .await
        .expect("node shutdown timed out")
        .unwrap()
        .unwrap();
    server.stop().wait().await;
    fs::remove_dir_all(root).unwrap();
}
