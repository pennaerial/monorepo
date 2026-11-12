use std::fmt;
use tungstenite::Bytes;

use crate::ros::server_types::{
    BinaryOpcode, CallId, ClientBinaryOpcode, MessageData, ServerMessage, ServiceCallResponse,
    ServiceId,
};

#[derive(Debug)]
pub enum ParseError {
    TooShort,
    InvalidOpcode(u8),
    InvalidEncoding,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => write!(f, "Incoming bin message too short for parsing"),
            Self::InvalidOpcode(opcode) => write!(f, "invalid opcode: {}", opcode),
            Self::InvalidEncoding => write!(f, "encoding name is not valid UTF-8"),
        }
    }
}

pub fn parse_binary_message(bytes: &Bytes) -> Result<ServerMessage, ParseError> {
    if bytes.is_empty() {
        return Err(ParseError::TooShort);
    }

    let mut offset = 0;
    let binary_opcode = BinaryOpcode::from_byte(bytes[offset])?;
    offset += 1;

    match binary_opcode {
        BinaryOpcode::MessageData => {
            let subscription_id: u32 = u32::from_le_bytes(
                bytes
                    .get(offset..offset + 4)
                    .ok_or(ParseError::TooShort)?
                    .try_into()
                    .unwrap(),
            );
            offset += 4;
            let timestamp: u64 = u64::from_le_bytes(
                bytes
                    .get(offset..offset + 8)
                    .ok_or(ParseError::TooShort)?
                    .try_into()
                    .unwrap(),
            );
            offset += 8;

            // get the rest of the bytes as a slice
            let data = bytes.slice(offset..);
            let message_data = MessageData {
                subscription_id,
                timestamp,
                data,
            };
            Ok(ServerMessage::MessageData(message_data))
        }
        BinaryOpcode::ServiceCallResponse => parse_service_call_response(bytes, offset),
        // TODO: add rest of the binary opcodes here
        //
        _ => Err(ParseError::InvalidOpcode(binary_opcode as u8)),
    }
}

fn read_u32(bytes: &Bytes, offset: &mut usize) -> Result<u32, ParseError> {
    let value = bytes
        .get(*offset..*offset + 4)
        .ok_or(ParseError::TooShort)?
        .try_into()
        .unwrap();
    *offset += 4;
    Ok(u32::from_le_bytes(value))
}

/// `[serviceId u32][callId u32][encodingLength u32][encoding][payload]`, after the opcode byte
fn parse_service_call_response(
    bytes: &Bytes,
    mut offset: usize,
) -> Result<ServerMessage, ParseError> {
    let service_id = read_u32(bytes, &mut offset)?;
    let call_id = read_u32(bytes, &mut offset)?;
    let encoding_length = read_u32(bytes, &mut offset)? as usize;
    let encoding_bytes = offset
        .checked_add(encoding_length)
        .and_then(|end| bytes.get(offset..end))
        .ok_or(ParseError::TooShort)?;
    let encoding = std::str::from_utf8(encoding_bytes)
        .map_err(|_| ParseError::InvalidEncoding)?
        .to_owned();
    offset += encoding_length;

    Ok(ServerMessage::ServiceCallResponse(ServiceCallResponse {
        service_id,
        call_id,
        encoding,
        data: bytes.slice(offset..),
    }))
}

/// Builds a binary service call request frame:
/// `[0x02][serviceId u32][callId u32][encodingLength u32][encoding][payload]`
pub fn encode_service_call_request(
    service_id: ServiceId,
    call_id: CallId,
    encoding: &str,
    payload: &[u8],
) -> Vec<u8> {
    let mut frame = Vec::with_capacity(1 + 4 * 3 + encoding.len() + payload.len());
    frame.push(ClientBinaryOpcode::ServiceCallRequest as u8);
    frame.extend_from_slice(&service_id.to_le_bytes());
    frame.extend_from_slice(&call_id.to_le_bytes());
    frame.extend_from_slice(&(encoding.len() as u32).to_le_bytes());
    frame.extend_from_slice(encoding.as_bytes());
    frame.extend_from_slice(payload);
    frame
}