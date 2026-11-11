use foxglove::{
    self,
    websocket::service::{Request, Service, ServiceSchema},
    FoxgloveError,
};
use std::error::Error;
use ros_interfaces::{RosMessage, RosService};

/// Utility type for running integration tests with a websocket
struct MockFoxgloveServer {
    server: foxglove::WebSocketServerHandle,
}

// no type enforcing on aliases, so
pub type ServiceCallback<S> = dyn Fn(<S as RosService>::Request) -> <S as RosService>::Response;

impl MockFoxgloveServer {
    pub async fn start() -> Result<Self, foxglove::FoxgloveError> {
        let server = foxglove::WebSocketServer::new()
            .bind("127.0.0.1", 0)
            .capabilities([foxglove::websocket::Capability::Services])
            .supported_encodings(["cdr"])
            .start()
            .await?;

        Ok(MockFoxgloveServer { server })
    }

    pub fn url(&self) -> String {
        format!("ws://127.0.0.1:{}", self.server.port())
    }

    pub async fn stop(self) {
        self.server.stop().wait().await;
    }

    /// generic method that bridges our RosService types to FoxgloveServer's Service building
    pub fn add_ros_server<S, E>(
        &self,
        service_name: &str,
        callback: impl Fn(S::Request) -> S::Response + Send + Sync + 'static,
    ) -> Result<(), FoxgloveError>
    where
        S: RosService,
        E: Error + Send + Sync + 'static,
    {
        // derive schema name from associated TYPE_NAME
        let schema = ServiceSchema::new(S::TYPE_NAME);
        let service = Service::builder(service_name, schema).handler_fn(
            // Under the hook, Ok() sends a ServiceCallResponse
            // Err() sends a ServiceCallFailure
            move |request| -> Result<Vec<u8>, Box<dyn Error + Send + Sync>> {
                let request = S::Request::deserialize(request.payload())?;
                let response = callback(request);
                Ok(response.serialize()?)
            },
        );
        self.server.add_services([service])?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::MockFoxgloveServer;

    #[tokio::test]
    async fn mock_server_start_stop() {
        let server = MockFoxgloveServer::start().await.unwrap();
        assert_ne!(server.server.port(), 0);
        assert_eq!(
            server.url(),
            format!("ws://127.0.0.1:{}", server.server.port())
        );
        server.stop().await;
    }
}
