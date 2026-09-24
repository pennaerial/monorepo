use std::fmt;
use crate::ros::server_types::ServerMessage;
use futures_util::{
    stream::{SplitSink, SplitStream},
    StreamExt,
    SinkExt,
};

use tokio::{net::TcpStream, sync::broadcast};
use tokio_tungstenite::{
    connect_async,
    tungstenite::{client::IntoClientRequest, http::HeaderValue, Message},
    MaybeTlsStream, WebSocketStream,
};
use serde_json::json;

pub type ServerMessageBroadcaster = broadcast::Sender<ServerMessage>;
pub type ServerMessageReceiver = broadcast::Receiver<ServerMessage>;

type Socket = WebSocketStream<MaybeTlsStream<TcpStream>>;
type SocketReader = SplitStream<Socket>;
type SocketWriter = SplitSink<Socket, Message>;

type SubscriptionId = u32;

const FOXGLOVE_SUBPROTOCOL: &str = "foxglove.sdk.v1";

#[derive(Debug)]
pub enum FoxgloveClientError {
    NotConnected,
    JsonError(serde_json::Error),
    WsError(tokio_tungstenite::tungstenite::Error),
}

// These 'impl From's let '?' automatically convert errors to the FoxgloveClientError enum
impl From<serde_json::Error> for FoxgloveClientError {
    fn from(error: serde_json::Error) -> Self {
        FoxgloveClientError::JsonError(error)
    }
}

impl From<tokio_tungstenite::tungstenite::Error> for FoxgloveClientError {
    fn from(error: tokio_tungstenite::tungstenite::Error) -> Self {
        FoxgloveClientError::WsError(error)
    }
}

// allows easy logging for debugging
impl fmt::Display for FoxgloveClientError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotConnected => write!(f, "FoxgloveClient NotConnected Error: Operation not permitted if FoxgloveClient is not connected to a bridge"),
            Self::JsonError(error) => write!(f, "FoxgloveClient JSON Error: {}", error),
            Self::WsError(error) => write!(f, "FoxgloveClient Websocket Error: {}", error),
        }
    }
}

// Invariant: if connected, then all resources are valid
pub struct FoxgloveClient {
    connected: bool,
    socket_writer: Option<SocketWriter>,
    broadcaster: ServerMessageBroadcaster,
    next_subscription_id: SubscriptionId,
}

impl FoxgloveClient {
    pub fn new() -> FoxgloveClient {
        let (broadcaster, _) = broadcast::channel(100); // 100 max buffer
        FoxgloveClient {
            connected: false,
            socket_writer: None,
            broadcaster,
            next_subscription_id: 0,
        }
    }

    pub async fn connect(&mut self, url: &str) -> Result<(), FoxgloveClientError> {
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
            if let Err(err) = FoxgloveClient::on_message_loop(broadcaster_clone, socket_reader).await {
                println!("Error occurred during on_message_loop: {err}");
            }
        });
        self.connected = true;
        Ok(())
    }

    // keep as associated function (no &mut self) bc we move all necessary resources into it
    async fn on_message_loop(broadcaster: ServerMessageBroadcaster, mut socket_reader: SocketReader) -> Result<(), FoxgloveClientError> {
        while let Some(msg) = socket_reader.next().await {
            let msg = msg?;
            match msg {
                Message::Text(text) => println!("{text}"),
                Message::Binary(bytes) => println!("Lossy string: {}", String::from_utf8_lossy(&bytes)),
                _ => (),
            }
            // TODO: use broadcaster for the correct ops/events
        }
        Ok(())
    }

    pub async fn subscribe(&mut self, channel_id: u32) -> Result<SubscriptionId, FoxgloveClientError> {
        if !self.connected { return Err(FoxgloveClientError::NotConnected); }
        let id = self.next_subscription_id;
        self.next_subscription_id += 1;
        let subscriptions = json!({
            "op": "subscribe",
            "subscriptions": [
                {
                    "id": id,
                    "channelId": channel_id
                }
            ]
        });
        let msg_str: String = serde_json::to_string(&subscriptions)?; // return JsonError error if fails
        let msg = Message::Text(msg_str.into());
        self.socket_writer.as_mut().unwrap().send(msg).await?; // return WsError if fails
        Ok(id)
    }


    pub fn listen_to_events(&mut self) -> Result<ServerMessageReceiver, FoxgloveClientError> {
        if !self.connected {
            return Err(FoxgloveClientError::NotConnected);
        }
        Ok(self.broadcaster.subscribe())
    }
}
