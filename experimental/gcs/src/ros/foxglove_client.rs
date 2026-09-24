use crate::ros::server_types::ServerMessage;
use futures_util::{
    stream::{SplitSink, SplitStream},
    StreamExt,
};
use std::time::Duration;
use tokio::time::sleep;
use tokio::{net::TcpStream, sync::broadcast};
use tokio_tungstenite::{
    connect_async,
    tungstenite::{client::IntoClientRequest, http::HeaderValue, Error, Message},
    MaybeTlsStream, WebSocketStream,
};

pub type ServerMessageBroadcaster = broadcast::Sender<ServerMessage>;
pub type ServerMessageReceiver = broadcast::Receiver<ServerMessage>;

type Socket = WebSocketStream<MaybeTlsStream<TcpStream>>;
type SocketReader = SplitStream<Socket>;
type SocketWriter = SplitSink<Socket, Message>;

const FOXGLOVE_SUBPROTOCOL: &str = "foxglove.sdk.v1";

// Invariant: if connected, then all resources are valid
pub struct FoxgloveClient {
    connected: bool,
    socket_writer: Option<SocketWriter>,
    broadcaster: ServerMessageBroadcaster,
}

impl FoxgloveClient {
    pub fn new() -> FoxgloveClient {
        let (broadcaster, _) = broadcast::channel(100); // 100 max buffer
        FoxgloveClient {
            connected: false,
            socket_writer: None,
            broadcaster,
        }
    }

    pub async fn connect(&mut self, url: &str) -> Result<(), Error> {
        let mut request = url.into_client_request()?;

        request.headers_mut().insert(
            "Sec-WebSocket-Protocol",
            HeaderValue::from_static(FOXGLOVE_SUBPROTOCOL),
        );

        let (socket, response) = connect_async(request).await?;
        let protocol = response
            .headers()
            .get("Sec-WebSocket-Protocol")
            .and_then(|value| value.to_str().ok())
            .unwrap_or("None");

        println!("Connection Success with subprotocol: {}", protocol);
        // self.socket = Some(socket);
        let (socket_writer, socket_reader) = socket.split();
        self.socket_writer = Some(socket_writer);

        // start the message handling loop
        let broadcaster_clone = self.broadcaster.clone();
        tokio::spawn(async move {
            FoxgloveClient::on_message_loop(broadcaster_clone, socket_reader).await;
        });
        Ok(())
    }

    // keep as associated function w/o self bc we move all necessary resources into it
    async fn on_message_loop(broadcaster: ServerMessageBroadcaster, mut socket_reader: SocketReader) {
        while let Some(msg) = socket_reader.next().await {
            match msg {
                Ok(msg) => println!("Received message {msg}"),
                Err(err) => println!("Got error {err}"),
            }
        }
    }

    pub fn listen_to_events(&mut self) -> ServerMessageReceiver {
        self.broadcaster.subscribe()
    }
}
