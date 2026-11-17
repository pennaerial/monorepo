use foxglove::{
    self,
    websocket::service::{Request, Service, ServiceSchema},
    ChannelBuilder, Context, FoxgloveError, RawChannel, Schema, WebSocketServerHandle,
};
use ros_interfaces::{RosMessage, RosService};
use std::error::Error;
use std::sync::Arc;

/// Utility type for running integration tests with a websocket
pub struct RosInterfaceServer {
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

        Ok(Self { server, context })
    }

    pub fn url(&self) -> String {
        format!("ws://127.0.0.1:{}", self.server.port())
    }

    pub async fn stop(self) {
        self.server.stop().wait().await;
    }

    /// generic method that bridges RosMessages to FoxgloveServer's Channel builder
    pub fn add_ros_channel<T: RosMessage>(
        &self,
        topic_name: &str,
    ) -> Result<Arc<RawChannel>, FoxgloveError> {
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
