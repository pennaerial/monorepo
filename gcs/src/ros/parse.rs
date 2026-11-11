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
            let subscription_id = read_u32(bytes, &mut offset)?;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_bytes_returns_too_short() {
        assert!(matches!(
            parse_binary_message(&Bytes::new()),
            Err(ParseError::TooShort)
        ));
    }

    #[test]
    fn invalid_opcode_returns_original_byte() {
        for opcode in 0u8..=u8::MAX {
            if matches!(opcode, 1 | 3) {
                continue;
            }

            let bytes = Bytes::copy_from_slice(&[opcode]);
            assert!(
                matches!(
                    parse_binary_message(&bytes),
                    Err(ParseError::InvalidOpcode(value)) if value == opcode
                ),
                "expected InvalidOpcode({opcode})"
            );
        }
    }

    #[test]
    fn message_data_with_short_subscription_id_returns_too_short() {
        let bytes = Bytes::from_static(&[1, 0x12, 0x34, 0x56]);
        assert!(matches!(
            parse_binary_message(&bytes),
            Err(ParseError::TooShort)
        ));
    }

    #[test]
    fn message_data_with_short_timestamp_returns_too_short() {
        let bytes = Bytes::from_static(&[
            1, // MessageData opcode
            1, 0, 0, 0, // subscription ID
            0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, // seven timestamp bytes
        ]);
        assert!(matches!(
            parse_binary_message(&bytes),
            Err(ParseError::TooShort)
        ));
    }

    #[test]
    fn parses_valid_message_data() {
        let bytes = Bytes::from_static(&[
            1, // MessageData opcode
            0x12, 0x34, 0x56, 0x78, // subscription ID
            0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, // timestamp
            0xaa, 0xbb, 0xcc, // payload
        ]);
        let ServerMessage::MessageData(message) = parse_binary_message(&bytes).unwrap() else {
            panic!("expected MessageData");
        };
        assert_eq!(message.subscription_id, 0x78563412);
        assert_eq!(message.timestamp, 0xefcdab8967452301);
        assert_eq!(message.data.as_ref(), &[0xaa, 0xbb, 0xcc]);
    }

    #[test]
    fn read_u32_with_short_input_returns_too_short() {
        let bytes = Bytes::from_static(&[0x12, 0x34, 0x56]);
        let mut offset = 0;
        assert!(matches!(read_u32(&bytes, &mut offset), Err(ParseError::TooShort)));
    }

    #[test]
    fn read_u32_correct_and_incr_offset() {
        let bytes = Bytes::from_static(&[0, 0x12, 0x34, 0x56, 0x78]);
        let mut offset = 1;
        assert_eq!(read_u32(&bytes, &mut offset).unwrap(), 0x78563412);
        assert_eq!(offset, 5);
    }

    #[test]
    fn parse_service_bad_service_id() {
        let bytes = Bytes::from_static(&[1, 0, 0]);
        assert!(matches!(
            parse_service_call_response(&bytes, 0),
            Err(ParseError::TooShort)
        ));
    }

    #[test]
    fn parse_service_bad_call_id() {
        let bytes = Bytes::from_static(&[1, 0, 0, 0, 2, 0, 0]);
        assert!(matches!(
            parse_service_call_response(&bytes, 0),
            Err(ParseError::TooShort)
        ));
    }

    #[test]
    fn parse_service_bad_encoding_length() {
        let bytes = Bytes::from_static(&[1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0]);
        assert!(matches!(
            parse_service_call_response(&bytes, 0),
            Err(ParseError::TooShort)
        ));
    }

    #[test]
    fn parse_service_bad_encoding_bytes() {
        let bytes = Bytes::from_static(&[1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, b'c', b'd']);
        assert!(matches!(
            parse_service_call_response(&bytes, 0),
            Err(ParseError::TooShort)
        ));
    }

    #[test]
    fn parse_service_invalid_encoding() {
        // No opcode: this calls the response parser directly at offset 0.
        let bytes = Bytes::from_static(&[
            1, 0, 0, 0, // service ID = 1
            2, 0, 0, 0, // call ID = 2
            1, 0, 0, 0, // encoding length = 1 byte
            0xff, // invalid UTF-8: 0xff is never a valid UTF-8 byte
        ]);
        assert!(matches!(
            parse_service_call_response(&bytes, 0),
            Err(ParseError::InvalidEncoding)
        ));
    }

    #[test]
    fn parses_valid_service_response() {
        let bytes = Bytes::from_static(&[
            3, // ServiceCallResponse opcode
            0x12, 0x34, 0x56, 0x78, // service ID
            0x01, 0x23, 0x45, 0x67, // call ID
            3, 0, 0, 0, b'c', b'd', b'r', // encoding length and encoding
            0xaa, 0xbb, // payload
        ]);
        let ServerMessage::ServiceCallResponse(response) = parse_binary_message(&bytes).unwrap() else {
            panic!("expected ServiceCallResponse");
        };
        assert_eq!(response.service_id, 0x78563412);
        assert_eq!(response.call_id, 0x67452301);
        assert_eq!(response.encoding, "cdr");
        assert_eq!(response.data.as_ref(), &[0xaa, 0xbb]);
    }

    #[test]
    fn encode_service_request_frame() {
        let frame = encode_service_call_request(0x78563412, 0x67452301, "cdr", &[0xaa, 0xbb]);
        assert_eq!(frame, [
            2, // ServiceCallRequest opcode
            0x12, 0x34, 0x56, 0x78, // service ID
            0x01, 0x23, 0x45, 0x67, // call ID
            3, 0, 0, 0, b'c', b'd', b'r', // encoding length and encoding
            0xaa, 0xbb, // payload
        ]);
    }
}
