mod testutils;

use gcs::ros::RosNode;
use testutils::RosInterfaceServer;
use tokio::time::{timeout, Duration};

#[tokio::test]
async fn node_connects_to_server() {
    let server = RosInterfaceServer::start().await.unwrap();
    let (handle, mut node) = RosNode::new(64);
    let task = tokio::spawn(async move { node.spin().await });

    timeout(Duration::from_secs(2), handle.connect(&server.url()))
        .await
        .expect("connection timed out")
        .expect("connection failed");

    drop(handle);
    timeout(Duration::from_secs(2), task)
        .await
        .expect("node shutdown timed out")
        .expect("node task panicked")
        .expect("node failed");
    server.stop().await;
}

#[tokio::test]
async fn subscription_receives_log() {
    use ros_interfaces::{builtin_interfaces::msg::Time, rcl_interfaces::msg::Log, RosMessage};
    use tokio::sync::mpsc;

    // Advertise /rosout and run the GCS node in a separate task.
    let server = RosInterfaceServer::start().await.unwrap();
    let channel = server.add_ros_channel::<Log>("/rosout").unwrap();
    let (handle, mut node) = RosNode::new(64);
    let task = tokio::spawn(async move { node.spin().await });
    // Forward callback messages to the test for comparison with this expected log.
    let (tx, mut rx) = mpsc::unbounded_channel();
    let expected = Log {
        stamp: Time { sec: 1, nanosec: 2 },
        level: Log::INFO,
        name: "test_node".into(),
        msg: "hello from the server".into(),
        file: "test.rs".into(),
        function: "test_log".into(),
        line: 42,
    };

    // Bound connection, subscription setup, and delivery with one timeout.
    let subscription = timeout(Duration::from_secs(2), async {
        handle.connect(&server.url()).await.unwrap();
        let subscription = handle
            .create_subscription::<Log>("/rosout", 10, move |message| {
                let _ = tx.send(message);
            })
            .await
            .unwrap();
        // Wait until the server has processed the subscription before publishing.
        while !channel.has_sinks() {
            tokio::task::yield_now().await;
        }
        // Publish CDR bytes and verify the callback receives the complete decoded message.
        channel.log(&expected.serialize().unwrap());
        assert_eq!(rx.recv().await.unwrap(), expected);
        subscription
    })
    .await
    .expect("log delivery timed out");

    // Stop the callback, then close the command channel so the node exits.
    drop(subscription);
    drop(handle);
    timeout(Duration::from_secs(2), task)
        .await
        .expect("node shutdown timed out")
        .unwrap()
        .unwrap();
    server.stop().await;
}
