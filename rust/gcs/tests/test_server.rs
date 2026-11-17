mod testutils;

use testutils::RosInterfaceServer;

#[tokio::test]
async fn mock_server_start_stop() {
    let server = RosInterfaceServer::start().await.unwrap();
    let address: std::net::SocketAddr =
        server.url().strip_prefix("ws://").unwrap().parse().unwrap();
    assert!(address.ip().is_loopback());
    assert_ne!(address.port(), 0);
    server.stop().await;
}

#[tokio::test]
async fn add_ros_service_registers_service() {
    use foxglove::FoxgloveError;
    use ros_interfaces::example_interfaces::srv::{AddTwoInts, AddTwoInts_Response};

    let server = RosInterfaceServer::start().await.unwrap();
    server
        .add_ros_service::<AddTwoInts>("/add_two_ints", |_| AddTwoInts_Response { sum: 5 })
        .unwrap();
    let duplicate =
        server.add_ros_service::<AddTwoInts>("/add_two_ints", |_| AddTwoInts_Response { sum: 5 });
    assert!(matches!(
        duplicate,
        Err(FoxgloveError::DuplicateService(name)) if name == "/add_two_ints"
    ));
    server.remove_service("/add_two_ints");
    server.stop().await;
}

#[tokio::test]
async fn add_ros_channel_creates_channel() {
    use ros_interfaces::{std_msgs::msg::Int32, RosMessage};

    let server = RosInterfaceServer::start().await.unwrap();
    let channel = server.add_ros_channel::<Int32>("/counter").unwrap();
    assert_eq!(channel.topic(), "/counter");
    assert_eq!(channel.message_encoding(), "cdr");
    assert_eq!(channel.schema().unwrap().name, Int32::TYPE_NAME);
    server.remove_channel("/counter");
    server.stop().await;
}
