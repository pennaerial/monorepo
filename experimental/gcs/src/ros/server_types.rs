use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub type ChannelId = u32;

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
    // Message,
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
