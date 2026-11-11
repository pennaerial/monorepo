use foxglove::{
    self, RawChannel, ChannelBuilder, Context, FoxgloveError, Schema, WebSocketServerHandle, websocket::service::{Request, Service, ServiceSchema},
};
use ros_interfaces::{RosMessage, RosService};
use std::error::Error;
use std::sync::Arc;

/// Utility type for running integration tests with a websocket
struct RosInterfaceServer {
    server: WebSocketServerHandle,
    context: Arc<Context>,
}


impl RosInterfaceServer {
    pub async fn start() -> Result<Self, foxglove::FoxgloveError> {
        let context = foxglove::Context::new();
        let server = foxglove::WebSocketServer::new()
            .context(&context)
            .bind("127.0.0.1", 0)
            .capabilities([foxglove::websocket::Capability::Services])
            .supported_encodings(["cdr"])
            .start()
            .await?;

        Ok(Self {
            server,
            context,
        })
    }

    pub fn url(&self) -> String {
        format!("ws://127.0.0.1:{}", self.server.port())
    }

    pub async fn stop(self) {
        self.server.stop().wait().await;
    }

    /// generic method that bridges RosMessages to FoxgloveServer's Channel builder
    pub fn add_ros_channel<T: RosMessage>(&self, topic_name: &str) -> Result<Arc<RawChannel>, FoxgloveError> {

        let dummy_schema = Schema::new(T::TYPE_NAME, "ros2msg", &b""[..]);

        let channel = ChannelBuilder::new(topic_name)
            .context(&self.context)
            .message_encoding("cdr")
            .schema(dummy_schema)
            .build_raw()?;

        Ok(channel)
    }

    /// generic method that bridges our RosService types to FoxgloveServer's Service builder
    pub fn add_ros_service<S: RosService>(
        &self,
        service_name: &str,
        callback: impl Fn(S::Request) -> S::Response + Send + Sync + 'static,
    ) -> Result<(), FoxgloveError> {
        // derive schema name from associated TYPE_NAME
        let schema = ServiceSchema::new(S::TYPE_NAME);
        let service = Service::builder(service_name, schema).handler_fn(
            // Under the hook, Ok() sends a ServiceCallResponse
            // Err() sends a ServiceCallFailure
            move |request: Request| -> Result<Vec<u8>, Box<dyn Error + Send + Sync>> {
                let request = S::Request::deserialize(request.payload())?;
                let response = callback(request);
                Ok(response.serialize()?)
            },
        );
        self.server.add_services([service])?;
        Ok(())
    }

    pub fn remove_service(&self, service_name: &str) {
        self.server.remove_services([service_name]);
    }

    pub fn remove_channel(&self, topic: &str) {
        if let Some(channel) = self.context.get_channel_by_topic(topic) {
            channel.close();
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::RosInterfaceServer;

    #[tokio::test]
    async fn mock_server_start_stop() {
        let server = RosInterfaceServer::start().await.unwrap();
        assert_ne!(server.server.port(), 0);
        assert_eq!(
            server.url(),
            format!("ws://127.0.0.1:{}", server.server.port())
        );
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
        let duplicate = server
            .add_ros_service::<AddTwoInts>("/add_two_ints", |_| AddTwoInts_Response { sum: 5 });
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
}
