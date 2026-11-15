use serde::Deserialize;
use std::collections::HashMap;
use std::fmt;
use tungstenite::Bytes;

use crate::ros::parse::ParseError;
use crate::ros::subscription::SubscriptionId;

pub type ChannelId = u32;
pub type ServiceId = u32;
/// Client-chosen id that pairs a service call request with its response
pub type CallId = u32;

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "op")]
pub enum ServerMessage {
    // Open,
    // Error,
    // Close,
    #[serde(rename = "serverInfo")]
    ServerInfo(ServerInfo),
    // Status,
    // RemoveStatus,
    // Time,
    #[serde(rename = "advertise")]
    Advertise(Advertise),
    // Unadvertise,
    #[serde(rename = "advertiseServices")]
    AdvertiseServices(AdvertiseServices),
    #[serde(rename = "unadvertiseServices")]
    UnadvertiseServices(UnadvertiseServices),
    // ParameterValues,
    // ConnectionGraphUpdate,
    // FetchAssetResponse,
    #[serde(rename = "serviceCallFailure")]
    ServiceCallFailure(ServiceCallFailure),
    // Binary ServerMessages
    #[serde(skip)]
    MessageData(MessageData),
    #[serde(skip)]
    ServiceCallResponse(ServiceCallResponse),
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ServerInfo {
    pub name: String,
    pub capabilities: Vec<String>,
    pub supported_encodings: Vec<String>,
    pub metadata: HashMap<String, String>,
    pub session_id: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Advertise {
    pub channels: Vec<Channel>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Channel {
    pub id: ChannelId,
    pub topic: String,
    pub encoding: String,
    pub schema_name: String,
    pub schema: String,
    pub schema_encoding: Option<String>,
}

impl fmt::Display for Channel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Channel ID: {:?}, Topic: {}, Schema: {}",
            self.id, self.topic, self.schema_name,
        )
    }
}

// Opcodes corresponding with each binary message
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinaryOpcode {
    MessageData = 1,
    Time = 2,
    ServiceCallResponse = 3,
    FetchAssetResponse = 4,
}

impl BinaryOpcode {
    pub fn from_byte(value: u8) -> Result<Self, ParseError> {
        match value {
            1 => Ok(Self::MessageData),
            2 => Ok(Self::Time),
            3 => Ok(Self::ServiceCallResponse),
            4 => Ok(Self::FetchAssetResponse),
            _ => Err(ParseError::InvalidOpcode(value)),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientBinaryOpcode {
    #[allow(dead_code)] // client publishing is not implemented yet
    MessageData = 1,
    ServiceCallRequest = 2,
}

// BINARY MESSAGES

#[derive(Debug, Clone)]
pub struct MessageData {
    // op: BinaryOpcode.MESSAGE_DATA;
    pub subscription_id: SubscriptionId,
    pub timestamp: u64,
    pub data: Bytes, // bytes moved from TcpStream
}

// SERVICES

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ServiceDefinition {
    pub encoding: String, // always use cdr for now
    pub schema_name: String,
    pub schema: String,
    pub schema_encoding: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Service {
    pub id: ServiceId,
    pub name: String,
    pub r#type: String,
    pub request: Option<ServiceDefinition>,
    pub response: Option<ServiceDefinition>,
    pub request_schema: Option<String>, // deprecated, but including in case
    pub response_schema: Option<String>,
}

impl fmt::Display for Service {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Service ID: {:?}, Name: {}, Type: {}",
            self.id, self.name, self.r#type,
        )
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct AdvertiseServices {
    pub services: Vec<Service>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnadvertiseServices {
    pub service_ids: Vec<ServiceId>,
}

/// Sent by the server when a service call could not be completed
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ServiceCallFailure {
    pub service_id: ServiceId,
    pub call_id: CallId,
    pub message: String,
}

/// Binary response to a service call request
#[derive(Debug, Clone)]
pub struct ServiceCallResponse {
    // op: BinaryOpcode.SERVICE_CALL_RESPONSE;
    pub service_id: ServiceId,
    pub call_id: CallId,
    pub encoding: String,
    pub data: Bytes, // CDR-encoded response
}
