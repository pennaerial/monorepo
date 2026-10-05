use std::fmt;
use tungstenite::Bytes;

use crate::ros::server_types::{MessageData, ServerMessage, BinaryOpcode};



#[derive(Debug)]
pub enum ParseError {
    TooShort,
    InvalidOpcode(u8),
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => write!(f, "Incoming bin message too short for parsing"),
            Self::InvalidOpcode(opcode) => write!(f, "invalid opcode: {}", opcode),
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
                .try_into().unwrap()
            );
            offset += 4;
            let timestamp: u64 = u64::from_le_bytes(
                bytes.get(offset..offset + 8)
                .ok_or(ParseError::TooShort)?
                .try_into().unwrap()
            );
            offset += 8;

            // get the rest of the bytes as a slice
            let data = bytes.slice(offset..);
            let message_data = MessageData{ subscription_id, timestamp, data };
            Ok(ServerMessage::MessageData(message_data))
        },
        // TODO: add rest of the binary opcodes here
        //
        _ => Err(ParseError::InvalidOpcode(binary_opcode as u8))
    }
}
