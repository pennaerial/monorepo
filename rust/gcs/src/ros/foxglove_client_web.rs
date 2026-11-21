use crate::ros::parse::{encode_service_call_request, parse_binary_message, ParseError};
use crate::ros::server_types::{CallId, ServerMessage, ServiceId};
use crate::ros::SubscriptionId;
use bytes::Bytes;
use js_sys::{Array, ArrayBuffer, Uint8Array};
use serde_json::json;
use std::{cell::Cell, fmt, rc::Rc};
use tokio::sync::{broadcast, oneshot};
use wasm_bindgen::{closure::Closure, JsCast};
use web_sys::{BinaryType, CloseEvent, ErrorEvent, Event, MessageEvent, WebSocket};

pub type ServerMessageBroadcaster = broadcast::Sender<ServerMessage>;
pub type ServerMessageReceiver = broadcast::Receiver<ServerMessage>;

const FOXGLOVE_SUBPROTOCOL: &str = "foxglove.sdk.v1";

#[derive(Debug)]
pub enum FoxgloveClientError {
    NotConnected,
    JsonError(serde_json::Error),
    BinError(ParseError),
    WsError(String),
}

impl From<serde_json::Error> for FoxgloveClientError {
    fn from(error: serde_json::Error) -> Self {
        Self::JsonError(error)
    }
}

impl From<ParseError> for FoxgloveClientError {
    fn from(error: ParseError) -> Self {
        Self::BinError(error)
    }
}

impl fmt::Display for FoxgloveClientError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotConnected => write!(f, "Foxglove client is not connected"),
            Self::JsonError(error) => write!(f, "Foxglove JSON error: {error}"),
            Self::BinError(error) => write!(f, "Foxglove binary error: {error}"),
            Self::WsError(error) => write!(f, "Foxglove WebSocket error: {error}"),
        }
    }
}

pub struct FoxgloveClient {
    socket: Option<WebSocket>,
    connected: Rc<Cell<bool>>,
    broadcaster: ServerMessageBroadcaster,
    next_subscription_id: SubscriptionId,
    next_call_id: CallId,
    on_message: Option<Closure<dyn FnMut(MessageEvent)>>,
    on_close: Option<Closure<dyn FnMut(CloseEvent)>>,
    on_error: Option<Closure<dyn FnMut(ErrorEvent)>>,
}

impl Default for FoxgloveClient {
    fn default() -> Self {
        Self::new()
    }
}

impl FoxgloveClient {
    pub fn new() -> Self {
        let (broadcaster, _) = broadcast::channel(100);
        Self {
            socket: None,
            connected: Rc::new(Cell::new(false)),
            broadcaster,
            next_subscription_id: 0,
            next_call_id: 0,
            on_message: None,
            on_close: None,
            on_error: None,
        }
    }

    pub async fn connect(&mut self, url: &str) -> Result<(), FoxgloveClientError> {
        let protocols = Array::new();
        protocols.push(&FOXGLOVE_SUBPROTOCOL.into());
        let socket = WebSocket::new_with_str_sequence(url, &protocols)
            .map_err(|error| FoxgloveClientError::WsError(format!("{error:?}")))?;
        socket.set_binary_type(BinaryType::Arraybuffer);

        let (opened_sender, opened_receiver) = oneshot::channel::<Result<(), String>>();
        let opened_sender = Rc::new(std::cell::RefCell::new(Some(opened_sender)));
        let open_sender = Rc::clone(&opened_sender);
        let on_open = Closure::<dyn FnMut(Event)>::new(move |_| {
            if let Some(sender) = open_sender.borrow_mut().take() {
                let _ = sender.send(Ok(()));
            }
        });
        let error_sender = Rc::clone(&opened_sender);
        let on_connect_error = Closure::<dyn FnMut(ErrorEvent)>::new(move |event: ErrorEvent| {
            if let Some(sender) = error_sender.borrow_mut().take() {
                let _ = sender.send(Err(event.message()));
            }
        });
        socket.set_onopen(Some(on_open.as_ref().unchecked_ref()));
        socket.set_onerror(Some(on_connect_error.as_ref().unchecked_ref()));
        opened_receiver
            .await
            .map_err(|_| FoxgloveClientError::WsError("connection callback dropped".into()))?
            .map_err(FoxgloveClientError::WsError)?;
        socket.set_onopen(None);

        if socket.protocol() != FOXGLOVE_SUBPROTOCOL {
            return Err(FoxgloveClientError::WsError(format!(
                "server selected unsupported subprotocol {}",
                socket.protocol()
            )));
        }

        let broadcaster = self.broadcaster.clone();
        let on_message = Closure::<dyn FnMut(MessageEvent)>::new(move |event: MessageEvent| {
            let parsed = if let Some(text) = event.data().as_string() {
                serde_json::from_str::<ServerMessage>(&text)
                    .map_err(|error| format!("failed to parse Foxglove JSON: {error}"))
            } else if let Ok(buffer) = event.data().dyn_into::<ArrayBuffer>() {
                let bytes = Bytes::from(Uint8Array::new(&buffer).to_vec());
                parse_binary_message(&bytes)
                    .map_err(|error| format!("failed to parse Foxglove binary data: {error}"))
            } else {
                Err("received unsupported Foxglove WebSocket message".into())
            };
            match parsed {
                Ok(message) => {
                    let _ = broadcaster.send(message);
                }
                Err(error) => dioxus::logger::tracing::error!("{error}"),
            }
        });
        socket.set_onmessage(Some(on_message.as_ref().unchecked_ref()));

        let connected = Rc::clone(&self.connected);
        let on_close =
            Closure::<dyn FnMut(CloseEvent)>::new(move |_: CloseEvent| connected.set(false));
        socket.set_onclose(Some(on_close.as_ref().unchecked_ref()));

        let on_error = Closure::<dyn FnMut(ErrorEvent)>::new(move |event: ErrorEvent| {
            dioxus::logger::tracing::error!("Foxglove WebSocket error: {}", event.message());
        });
        socket.set_onerror(Some(on_error.as_ref().unchecked_ref()));

        self.connected.set(true);
        self.socket = Some(socket);
        self.on_message = Some(on_message);
        self.on_close = Some(on_close);
        self.on_error = Some(on_error);
        Ok(())
    }

    pub async fn subscribe(
        &mut self,
        channel_id: u32,
    ) -> Result<SubscriptionId, FoxgloveClientError> {
        let id = self.next_subscription_id;
        self.next_subscription_id = self.next_subscription_id.wrapping_add(1);
        let message = serde_json::to_string(&json!({
            "op": "subscribe",
            "subscriptions": [{"id": id, "channelId": channel_id}]
        }))?;
        self.socket()?.send_with_str(&message).map_err(js_error)?;
        Ok(id)
    }

    pub async fn call_service(
        &mut self,
        service_id: ServiceId,
        encoding: &str,
        payload: &[u8],
    ) -> Result<CallId, FoxgloveClientError> {
        let call_id = self.next_call_id;
        self.next_call_id = self.next_call_id.wrapping_add(1);
        let frame = encode_service_call_request(service_id, call_id, encoding, payload);
        self.socket()?
            .send_with_u8_array(&frame)
            .map_err(js_error)?;
        Ok(call_id)
    }

    pub fn get_receiver(&mut self) -> ServerMessageReceiver {
        self.broadcaster.subscribe()
    }

    pub fn is_connected(&self) -> bool {
        self.connected.get()
    }

    fn socket(&self) -> Result<&WebSocket, FoxgloveClientError> {
        if !self.is_connected() {
            return Err(FoxgloveClientError::NotConnected);
        }
        self.socket
            .as_ref()
            .ok_or(FoxgloveClientError::NotConnected)
    }
}

fn js_error(error: wasm_bindgen::JsValue) -> FoxgloveClientError {
    FoxgloveClientError::WsError(format!("{error:?}"))
}
