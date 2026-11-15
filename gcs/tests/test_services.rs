mod testutils;

use gcs::ros::{NodeError, RosNode};
use ros_interfaces::example_interfaces::srv::{
    AddTwoInts, AddTwoInts_Request, AddTwoInts_Response,
};
use testutils::RosInterfaceServer;
use tokio::time::{timeout, Duration};

#[tokio::test]
async fn service_adds_two_ints() {
    let server = RosInterfaceServer::start().await.unwrap(); // create mock server

    // Add AddTwoInts service server, with expected (2, 3) request input
    server
        .add_ros_service::<AddTwoInts>("/add_two_ints", |request| {
            assert_eq!((request.a, request.b), (2, 3));
            AddTwoInts_Response {
                sum: request.a + request.b,
            }
        })
        .unwrap();

    // create RosNode and pin in task
    let (handle, mut node) = RosNode::new(64);
    let task = tokio::spawn(async move { node.spin().await });

    // bound follow async block for 2 seconds
    timeout(Duration::from_secs(2), async {
        handle.connect(&server.url()).await.unwrap();
        let request = AddTwoInts_Request { a: 2, b: 3 };
        let response = loop {
            match handle
                .call_service::<AddTwoInts>("/add_two_ints", &request, Duration::from_secs(1))
                .await
            {
                // Connection can finish before the node processes the advertisement.
                Err(NodeError::ServiceNotAdvertised(_)) => tokio::task::yield_now().await,
                result => break result.unwrap(),
            }
        };
        assert_eq!(response.sum, 5);
    })
    .await
    .expect("service call timed out");

    // shutdown safely
    // Close the command channel and wait for the node to exit.
    drop(handle);
    timeout(Duration::from_secs(2), task)
        .await
        .expect("node shutdown timed out")
        .unwrap()
        .unwrap();
    server.stop().await;
}
