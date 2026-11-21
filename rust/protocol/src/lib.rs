use hmac::{Hmac, Mac};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const PROTOCOL_VERSION: u16 = 1;
pub const DEFAULT_AGENT_PORT: u16 = 8765;
pub const MAX_FILE_CHUNK_SIZE: usize = 64 * 1024;
pub const FOXGLOVE_JSON_ENCODING: &str = "json";
pub const FOXGLOVE_FILE_CHUNK_ENCODING: &str = "pennair.file-chunk.v1";
pub const FOXGLOVE_SERVICE_TYPE: &str = "pennair.deployment.v1";
pub const SERVICE_HELLO: &str = "/pennair/deployment/hello";
pub const SERVICE_AUTHENTICATE: &str = "/pennair/deployment/authenticate";
pub const SERVICE_HEALTH: &str = "/pennair/deployment/health";
pub const SERVICE_INVENTORY: &str = "/pennair/deployment/inventory";
pub const SERVICE_PREPARE: &str = "/pennair/deployment/prepare";
pub const SERVICE_BEGIN_FILE: &str = "/pennair/deployment/begin_file";
pub const SERVICE_WRITE_CHUNK: &str = "/pennair/deployment/write_chunk";
pub const SERVICE_COMPLETE_FILE: &str = "/pennair/deployment/complete_file";
pub const SERVICE_ACTIVATE: &str = "/pennair/deployment/activate";
pub const SERVICE_BUILD: &str = "/pennair/deployment/build";
pub const SERVICE_RESTART: &str = "/pennair/deployment/restart";
const FILE_CHUNK_MAGIC: &[u8; 4] = b"PNFC";
const FILE_CHUNK_HEADER_SIZE: usize = 22;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Request {
    pub protocol_version: u16,
    pub request_id: u64,
    pub operation: RequestOperation,
}

impl Request {
    pub fn new(request_id: u64, operation: RequestOperation) -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            request_id,
            operation,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", content = "data", rename_all = "snake_case")]
pub enum RequestOperation {
    Hello {
        client_name: String,
    },
    Authenticate {
        proof: String,
    },
    Health,
    DeploymentInventory,
    PrepareDeployment {
        manifest: DeploymentManifest,
        proof: String,
    },
    BeginFile {
        deployment_id: String,
        path: String,
    },
    CompleteFile {
        deployment_id: String,
        path: String,
    },
    ActivateDeployment {
        deployment_id: String,
        proof: String,
    },
    BuildDeployment {
        deployment_id: String,
        proof: String,
    },
    RestartService {
        deployment_id: String,
        proof: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Response {
    pub protocol_version: u16,
    pub request_id: u64,
    pub result: ResponseResult,
}

impl Response {
    pub fn new(request_id: u64, result: ResponseResult) -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            request_id,
            result,
        }
    }

    pub fn error(request_id: u64, code: impl Into<String>, message: impl Into<String>) -> Self {
        Self::new(
            request_id,
            ResponseResult::Error {
                code: code.into(),
                message: message.into(),
            },
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", content = "data", rename_all = "snake_case")]
pub enum ResponseResult {
    Hello {
        agent_name: String,
        auth_challenge: String,
    },
    Authenticated,
    Health {
        agent_name: String,
        uptime_seconds: u64,
    },
    DeploymentInventory {
        deployment_id: Option<String>,
        files: Vec<String>,
    },
    DeploymentPrepared {
        deployment_id: String,
        required_files: Vec<String>,
    },
    FileReady {
        path: String,
        offset: u64,
    },
    ChunkAccepted {
        path: String,
        next_offset: u64,
    },
    FileCompleted {
        path: String,
    },
    DeploymentActivated {
        deployment_id: String,
        files: usize,
        deletions: usize,
    },
    BuildOutput {
        stream: OutputStream,
        line: String,
    },
    BuildCompleted {
        deployment_id: String,
        success: bool,
        exit_code: Option<i32>,
        output: Vec<BuildOutputLine>,
    },
    ServiceRestarted {
        deployment_id: String,
        service: String,
    },
    Error {
        code: String,
        message: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeploymentManifest {
    pub deployment_id: String,
    pub files: Vec<DeploymentFile>,
    pub deletions: Vec<String>,
    pub restart: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeploymentFile {
    pub path: String,
    pub size: u64,
    pub sha256: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputStream {
    Stdout,
    Stderr,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BuildOutputLine {
    pub stream: OutputStream,
    pub line: String,
}

pub struct FileChunk<'a> {
    pub request_id: u64,
    pub offset: u64,
    pub data: &'a [u8],
}

pub fn encode_file_chunk(request_id: u64, offset: u64, data: &[u8]) -> Result<Vec<u8>, String> {
    if data.is_empty() || data.len() > MAX_FILE_CHUNK_SIZE {
        return Err(format!(
            "file chunks must contain between 1 and {MAX_FILE_CHUNK_SIZE} bytes"
        ));
    }
    let mut frame = Vec::with_capacity(FILE_CHUNK_HEADER_SIZE + data.len());
    frame.extend_from_slice(FILE_CHUNK_MAGIC);
    frame.extend_from_slice(&PROTOCOL_VERSION.to_be_bytes());
    frame.extend_from_slice(&request_id.to_be_bytes());
    frame.extend_from_slice(&offset.to_be_bytes());
    frame.extend_from_slice(data);
    Ok(frame)
}

pub fn decode_file_chunk(frame: &[u8]) -> Result<FileChunk<'_>, String> {
    if frame.len() <= FILE_CHUNK_HEADER_SIZE {
        return Err("file chunk is missing its header or payload".into());
    }
    if &frame[..4] != FILE_CHUNK_MAGIC {
        return Err("invalid file chunk magic".into());
    }
    let version = u16::from_be_bytes(frame[4..6].try_into().unwrap());
    if version != PROTOCOL_VERSION {
        return Err(format!(
            "unsupported file chunk protocol {version}; expected {PROTOCOL_VERSION}"
        ));
    }
    let data = &frame[FILE_CHUNK_HEADER_SIZE..];
    if data.len() > MAX_FILE_CHUNK_SIZE {
        return Err(format!(
            "file chunk exceeds the {MAX_FILE_CHUNK_SIZE}-byte limit"
        ));
    }
    Ok(FileChunk {
        request_id: u64::from_be_bytes(frame[6..14].try_into().unwrap()),
        offset: u64::from_be_bytes(frame[14..22].try_into().unwrap()),
        data,
    })
}

pub fn authentication_proof(token: &str, message: &str) -> String {
    let mut mac =
        Hmac::<Sha256>::new_from_slice(token.as_bytes()).expect("HMAC accepts keys of any length");
    mac.update(message.as_bytes());
    hex::encode(mac.finalize().into_bytes())
}

pub fn verify_authentication_proof(token: &str, message: &str, proof: &str) -> bool {
    let Ok(proof) = hex::decode(proof) else {
        return false;
    };
    let mut mac =
        Hmac::<Sha256>::new_from_slice(token.as_bytes()).expect("HMAC accepts keys of any length");
    mac.update(message.as_bytes());
    mac.verify_slice(&proof).is_ok()
}

pub fn deployment_proof_message(challenge: &str, deployment_id: &str) -> String {
    format!("{challenge}:deploy:{deployment_id}")
}

pub fn activation_proof_message(challenge: &str, deployment_id: &str) -> String {
    format!("{challenge}:activate:{deployment_id}")
}

pub fn build_proof_message(challenge: &str, deployment_id: &str) -> String {
    format!("{challenge}:build:{deployment_id}")
}

pub fn restart_proof_message(challenge: &str, deployment_id: &str) -> String {
    format!("{challenge}:restart:{deployment_id}")
}

pub fn foxglove_service_for_operation(operation: &RequestOperation) -> &'static str {
    match operation {
        RequestOperation::Hello { .. } => SERVICE_HELLO,
        RequestOperation::Authenticate { .. } => SERVICE_AUTHENTICATE,
        RequestOperation::Health => SERVICE_HEALTH,
        RequestOperation::DeploymentInventory => SERVICE_INVENTORY,
        RequestOperation::PrepareDeployment { .. } => SERVICE_PREPARE,
        RequestOperation::BeginFile { .. } => SERVICE_BEGIN_FILE,
        RequestOperation::CompleteFile { .. } => SERVICE_COMPLETE_FILE,
        RequestOperation::ActivateDeployment { .. } => SERVICE_ACTIVATE,
        RequestOperation::BuildDeployment { .. } => SERVICE_BUILD,
        RequestOperation::RestartService { .. } => SERVICE_RESTART,
    }
}

pub fn foxglove_json_service_names() -> &'static [&'static str] {
    &[
        SERVICE_HELLO,
        SERVICE_AUTHENTICATE,
        SERVICE_HEALTH,
        SERVICE_INVENTORY,
        SERVICE_PREPARE,
        SERVICE_BEGIN_FILE,
        SERVICE_COMPLETE_FILE,
        SERVICE_ACTIVATE,
        SERVICE_BUILD,
        SERVICE_RESTART,
    ]
}

pub fn calculate_deployment_id(
    files: &[DeploymentFile],
    deletions: &[String],
    restart: bool,
) -> String {
    let mut files = files.to_vec();
    files.sort_by(|a, b| a.path.cmp(&b.path));
    let mut deletions = deletions.to_vec();
    deletions.sort();

    let mut hasher = Sha256::new();
    hasher.update(b"pennair-deployment-v1");
    hasher.update([u8::from(restart)]);
    for file in files {
        hasher.update(b"file");
        update_hash_field(&mut hasher, file.path.as_bytes());
        hasher.update(file.size.to_be_bytes());
        update_hash_field(&mut hasher, file.sha256.as_bytes());
    }
    for path in deletions {
        hasher.update(b"delete");
        update_hash_field(&mut hasher, path.as_bytes());
    }
    hex::encode(hasher.finalize())
}

pub fn validate_deployment_manifest(manifest: &DeploymentManifest) -> Result<(), String> {
    if manifest.files.len() > 100_000 {
        return Err("manifest contains more than 100000 files".into());
    }
    if !is_sha256(&manifest.deployment_id) {
        return Err("deployment ID must be a lowercase SHA-256 digest".into());
    }

    let mut paths = BTreeSet::new();
    for file in &manifest.files {
        validate_relative_path(&file.path)?;
        if !is_sha256(&file.sha256) {
            return Err(format!("invalid SHA-256 digest for {}", file.path));
        }
        if !paths.insert(file.path.as_str()) {
            return Err(format!("duplicate manifest path: {}", file.path));
        }
    }
    for path in &manifest.deletions {
        validate_relative_path(path)?;
        if !paths.insert(path) {
            return Err(format!("duplicate manifest path: {path}"));
        }
    }

    let expected = calculate_deployment_id(&manifest.files, &manifest.deletions, manifest.restart);
    if manifest.deployment_id != expected {
        return Err("deployment ID does not match the manifest contents".into());
    }
    Ok(())
}

fn validate_relative_path(path: &str) -> Result<(), String> {
    if path.is_empty()
        || path.len() > 4096
        || path.starts_with('/')
        || path.contains('\\')
        || path == ".pennair"
        || path.starts_with(".pennair/")
        || path
            .split('/')
            .any(|part| part.is_empty() || part == "." || part == "..")
    {
        return Err(format!("unsafe deployment path: {path}"));
    }
    Ok(())
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn update_hash_field(hasher: &mut Sha256, value: &[u8]) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_round_trips_as_json() {
        let request = Request::new(
            42,
            RequestOperation::Hello {
                client_name: "pennair-cli".into(),
            },
        );

        let json = serde_json::to_string(&request).unwrap();
        let decoded = serde_json::from_str(&json).unwrap();

        assert_eq!(request, decoded);
    }

    #[test]
    fn error_response_round_trips_as_json() {
        let response = Response::error(7, "handshake_required", "send hello first");

        let json = serde_json::to_string(&response).unwrap();
        let decoded = serde_json::from_str(&json).unwrap();

        assert_eq!(response, decoded);
    }

    #[test]
    fn authentication_proof_verifies_without_sending_the_token() {
        let proof = authentication_proof("a sufficiently long test token", "random challenge");

        assert!(verify_authentication_proof(
            "a sufficiently long test token",
            "random challenge",
            &proof
        ));
        assert!(!verify_authentication_proof(
            "wrong token",
            "random challenge",
            &proof
        ));
    }

    #[test]
    fn manifest_rejects_parent_directory_paths() {
        let files = vec![DeploymentFile {
            path: "../outside".into(),
            size: 1,
            sha256: "a".repeat(64),
        }];
        let manifest = DeploymentManifest {
            deployment_id: calculate_deployment_id(&files, &[], false),
            files,
            deletions: vec![],
            restart: false,
        };

        assert!(validate_deployment_manifest(&manifest).is_err());
    }

    #[test]
    fn manifest_rejects_agent_control_paths() {
        let files = vec![DeploymentFile {
            path: ".pennair/current.json".into(),
            size: 1,
            sha256: "a".repeat(64),
        }];
        let manifest = DeploymentManifest {
            deployment_id: calculate_deployment_id(&files, &[], false),
            files,
            deletions: vec![],
            restart: false,
        };

        assert!(validate_deployment_manifest(&manifest).is_err());
    }

    #[test]
    fn valid_manifest_has_deterministic_id() {
        let files = vec![DeploymentFile {
            path: "ros_ws/src/package/file.py".into(),
            size: 12,
            sha256: "b".repeat(64),
        }];
        let deployment_id = calculate_deployment_id(&files, &[], true);
        let manifest = DeploymentManifest {
            deployment_id: deployment_id.clone(),
            files,
            deletions: vec![],
            restart: true,
        };

        assert_eq!(deployment_id.len(), 64);
        assert_eq!(validate_deployment_manifest(&manifest), Ok(()));
    }

    #[test]
    fn binary_file_chunk_round_trips() {
        let encoded = encode_file_chunk(19, 65_536, b"file bytes").unwrap();
        let decoded = decode_file_chunk(&encoded).unwrap();

        assert_eq!(decoded.request_id, 19);
        assert_eq!(decoded.offset, 65_536);
        assert_eq!(decoded.data, b"file bytes");
    }

    #[test]
    fn oversized_binary_chunk_is_rejected() {
        let bytes = vec![0; MAX_FILE_CHUNK_SIZE + 1];

        assert!(encode_file_chunk(1, 0, &bytes).is_err());
    }

    #[test]
    fn operations_map_to_stable_foxglove_service_names() {
        assert_eq!(
            foxglove_service_for_operation(&RequestOperation::Health),
            SERVICE_HEALTH
        );
        assert_eq!(
            foxglove_service_for_operation(&RequestOperation::BeginFile {
                deployment_id: "deployment".into(),
                path: "file".into(),
            }),
            SERVICE_BEGIN_FILE
        );
        assert!(!foxglove_json_service_names().contains(&SERVICE_WRITE_CHUNK));
    }
}
