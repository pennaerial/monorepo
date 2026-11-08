use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;
use tungstenite::Bytes;

use crate::ros::parse::ParseError;
use crate::ros::subscription::SubscriptionId;

pub type ChannelId = u32;
pub type ServiceId = u32;

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
    #[serde(rename="advertiseServices")]
    AdvertiseServices(AdvertiseServices),
    #[serde(rename="unadvertiseServices")]
    UnadvertiseServices(UnadvertiseServices),
    // ParameterValues,
    // ServiceCallResponse,
    // ConnectionGraphUpdate,
    // FetchAssetResponse,
    // ServiceCallFailure,
    // Binary ServerMessages
    #[serde(skip)]
    MessageData(MessageData),
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Advertise {
    pub channels: Vec<Channel>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Channel {
    pub id: ChannelId,
    pub topic: String,
    pub encoding: String,
    pub schema_name: String,
    pub schema: String,
    #[serde(skip_serializing_if = "Option::is_none")]
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

// BINARY MESSAGES

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageData {
    // op: BinaryOpcode.MESSAGE_DATA;
    pub subscription_id: SubscriptionId,
    pub timestamp: u64,
    pub data: Bytes, // bytes moved from TcpStream
}

// SERVICES

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ServiceDefinition {
    pub encoding: String,
    pub schema_name: String,
    pub schema: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub schema_encoding: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvertiseServices {
    pub services: Vec<Service>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnadvertiseServices {
    pub service_ids: Vec<ServiceId>,
}
