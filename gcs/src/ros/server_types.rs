use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use tungstenite::Bytes;

use crate::ros::parse::ParseError;

pub type ChannelId = u32;
pub type SubscriptionId = u32;

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
    // AdvertiseServices,
    // UnadvertiseServices,
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
    pub session_id: i32,
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
