use foxglove;

/// Utility type for running integration tests with a websocket
struct MockFoxgloveServer {
    server: foxglove::WebSocketServerHandle,
}

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

    pub async fn stop(self) {
        self.server.stop().wait().await;
    }
}

#[cfg(test)]
mod tests {
    use crate::MockFoxgloveServer;

    #[tokio::test]
    async fn mock_server_start_stop() {
        let server = MockFoxgloveServer::start().await.unwrap();
        assert_ne!(server.server.port(), 0);

        server.stop().await;
    }
}
