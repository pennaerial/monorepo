use crate::config::AgentConfig;
use foxglove::websocket::service::{ClientId, Request as FoxgloveRequest, Service, ServiceSchema};
use foxglove::{Context, Schema, WebSocketServer, WebSocketServerHandle};
use pennair_protocol::{
    BuildOutputLine, DeploymentFile, DeploymentManifest, FOXGLOVE_FILE_CHUNK_ENCODING,
    FOXGLOVE_JSON_ENCODING, FOXGLOVE_SERVICE_TYPE, OutputStream, PROTOCOL_VERSION, Request,
    RequestOperation, Response, ResponseResult, SERVICE_WRITE_CHUNK, activation_proof_message,
    build_proof_message, decode_file_chunk, deployment_proof_message, foxglove_json_service_names,
    foxglove_service_for_operation, restart_proof_message, validate_deployment_manifest,
    verify_authentication_proof,
};
use rand::RngCore;
use sha2::{Digest, Sha256};
use std::{
    collections::{HashMap, HashSet},
    error::Error,
    fs::{self, File, OpenOptions},
    io::{BufRead, BufReader, Read, Seek, SeekFrom, Write},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::{Arc, Mutex, mpsc},
    thread,
    time::Instant,
};

pub struct AgentServer {
    config: AgentConfig,
    started_at: Instant,
    context: Option<Arc<Context>>,
}

impl AgentServer {
    pub fn new(config: AgentConfig) -> Self {
        Self {
            config,
            started_at: Instant::now(),
            context: None,
        }
    }

    /// Uses an existing Foxglove context so telemetry channels and deployment
    /// services are advertised by the same server.
    pub fn with_context(mut self, context: Arc<Context>) -> Self {
        self.context = Some(context);
        self
    }

    pub fn run(self) -> Result<(), Box<dyn Error>> {
        let (handle, agent_name, bind_address) = self.start()?;
        println!(
            "pennair-agent {agent_name} listening on Foxglove WebSocket ws://{}:{}",
            bind_address.ip(),
            handle.port()
        );
        loop {
            thread::park();
        }
    }

    pub fn start(
        self,
    ) -> Result<(WebSocketServerHandle, String, std::net::SocketAddr), Box<dyn Error>> {
        let (server, agent_name, bind_address) = self.configured_server()?;
        let handle = server.start_blocking()?;
        Ok((handle, agent_name, bind_address))
    }

    pub async fn start_async(
        self,
    ) -> Result<(WebSocketServerHandle, String, std::net::SocketAddr), Box<dyn Error>> {
        let (server, agent_name, bind_address) = self.configured_server()?;
        let handle = server.start().await?;
        Ok((handle, agent_name, bind_address))
    }

    fn configured_server(
        self,
    ) -> Result<(WebSocketServer, String, std::net::SocketAddr), Box<dyn Error>> {
        initialize_deployment_root(&self.config.deployment_root)?;
        let bind_address = self.config.bind_address;
        let agent_name = self.config.agent_name.clone();

        let state = Arc::new(ServerState {
            agent_name: self.config.agent_name,
            token: self.config.token,
            staging_root: self.config.deployment_root.join(".pennair/staging"),
            deployment_root: self.config.deployment_root,
            build_shell: self.config.build_shell,
            ros_setup: self.config.ros_setup,
            colcon: self.config.colcon,
            sudo: self.config.sudo,
            systemctl: self.config.systemctl,
            service_name: self.config.service_name,
            active_files: Mutex::new(HashSet::new()),
            activation_lock: Mutex::new(()),
            sessions: Mutex::new(HashMap::new()),
            started_at: self.started_at,
        });
        let services = deployment_services(state);
        let mut server = WebSocketServer::new()
            .name(format!("PennAir {agent_name}"))
            .bind(bind_address.ip().to_string(), bind_address.port())
            .supported_encodings([FOXGLOVE_JSON_ENCODING, FOXGLOVE_FILE_CHUNK_ENCODING])
            .services(services);
        if let Some(context) = self.context {
            server = server.context(&context);
        }
        Ok((server, agent_name, bind_address))
    }
}

struct ServerState {
    agent_name: String,
    token: String,
    staging_root: PathBuf,
    deployment_root: PathBuf,
    build_shell: PathBuf,
    ros_setup: PathBuf,
    colcon: PathBuf,
    sudo: PathBuf,
    systemctl: PathBuf,
    service_name: String,
    active_files: Mutex<HashSet<String>>,
    activation_lock: Mutex<()>,
    sessions: Mutex<HashMap<ClientId, Session>>,
    started_at: Instant,
}

#[derive(Default)]
struct Session {
    greeted: bool,
    authenticated: bool,
    challenge: Option<String>,
    pending_deployment: Option<DeploymentManifest>,
    active_transfer: Option<ActiveTransfer>,
}

struct ActiveTransfer {
    key: String,
    file: DeploymentFile,
    partial_path: PathBuf,
    final_path: PathBuf,
    received: u64,
}

fn deployment_services(state: Arc<ServerState>) -> Vec<Service> {
    let request_schema = Schema::new(
        "pennair.deployment.Request",
        "jsonschema",
        br#"{"type":"object"}"#,
    );
    let response_schema = Schema::new(
        "pennair.deployment.Response",
        "jsonschema",
        br#"{"type":"object"}"#,
    );
    let mut services = Vec::new();
    for &name in foxglove_json_service_names() {
        let schema = ServiceSchema::new(FOXGLOVE_SERVICE_TYPE)
            .with_request(FOXGLOVE_JSON_ENCODING, request_schema.clone())
            .with_response(FOXGLOVE_JSON_ENCODING, response_schema.clone());
        let service_state = Arc::clone(&state);
        services.push(
            Service::builder(name, schema).blocking_handler_fn(move |request| {
                Ok::<Vec<u8>, String>(handle_foxglove_json_request(request, &service_state))
            }),
        );
    }
    let chunk_schema = ServiceSchema::new(FOXGLOVE_SERVICE_TYPE)
        .with_request(
            FOXGLOVE_FILE_CHUNK_ENCODING,
            Schema::new("pennair.deployment.FileChunk", "", &[][..]),
        )
        .with_response(FOXGLOVE_JSON_ENCODING, response_schema);
    services.push(
        Service::builder(SERVICE_WRITE_CHUNK, chunk_schema).blocking_handler_fn(move |request| {
            Ok::<Vec<u8>, String>(handle_foxglove_chunk_request(request, &state))
        }),
    );
    services
}

fn handle_foxglove_json_request(request: FoxgloveRequest, state: &ServerState) -> Vec<u8> {
    let client_id = request.client_id();
    let service_name = request.service_name().to_owned();
    let response = if request.encoding() != FOXGLOVE_JSON_ENCODING {
        Response::error(
            0,
            "invalid_encoding",
            format!(
                "expected {FOXGLOVE_JSON_ENCODING}, received {}",
                request.encoding()
            ),
        )
    } else {
        match serde_json::from_slice::<Request>(request.payload()) {
            Ok(request) if foxglove_service_for_operation(&request.operation) != service_name => {
                Response::error(
                    request.request_id,
                    "service_mismatch",
                    format!(
                        "{} requests must use {}",
                        service_name,
                        foxglove_service_for_operation(&request.operation)
                    ),
                )
            }
            Ok(request) => {
                let mut sessions = state.sessions.lock().unwrap();
                let session = sessions.entry(client_id).or_default();
                if matches!(&request.operation, RequestOperation::BuildDeployment { .. }) {
                    handle_build_request(request, session, state)
                } else {
                    dispatch(request, session, state)
                }
            }
            Err(error) => Response::error(0, "invalid_request", error.to_string()),
        }
    };
    serialize_response(response)
}

fn handle_foxglove_chunk_request(request: FoxgloveRequest, state: &ServerState) -> Vec<u8> {
    let response = if request.encoding() != FOXGLOVE_FILE_CHUNK_ENCODING {
        Response::error(
            0,
            "invalid_encoding",
            format!(
                "expected {FOXGLOVE_FILE_CHUNK_ENCODING}, received {}",
                request.encoding()
            ),
        )
    } else {
        let mut sessions = state.sessions.lock().unwrap();
        let session = sessions.entry(request.client_id()).or_default();
        handle_file_chunk(request.payload(), session)
    };
    serialize_response(response)
}

fn serialize_response(response: Response) -> Vec<u8> {
    serde_json::to_vec(&response).unwrap_or_else(|error| {
        format!(
            "{{\"protocol_version\":{PROTOCOL_VERSION},\"request_id\":0,\"result\":{{\"type\":\"error\",\"data\":{{\"code\":\"serialization_error\",\"message\":{}}}}}}}",
            serde_json::to_string(&error.to_string()).unwrap()
        )
        .into_bytes()
    })
}

fn dispatch(request: Request, session: &mut Session, state: &ServerState) -> Response {
    if request.protocol_version != PROTOCOL_VERSION {
        return Response::error(
            request.request_id,
            "unsupported_protocol",
            format!(
                "agent supports protocol {PROTOCOL_VERSION}, received {}",
                request.protocol_version
            ),
        );
    }

    match request.operation {
        RequestOperation::Hello { .. } => {
            release_active_transfer(session, state);
            let challenge = random_challenge();
            session.greeted = true;
            session.authenticated = false;
            session.challenge = Some(challenge.clone());
            session.pending_deployment = None;
            Response::new(
                request.request_id,
                ResponseResult::Hello {
                    agent_name: state.agent_name.clone(),
                    auth_challenge: challenge,
                },
            )
        }
        RequestOperation::Authenticate { .. } if !session.greeted => Response::error(
            request.request_id,
            "handshake_required",
            "send hello before authentication",
        ),
        RequestOperation::Authenticate { proof } => {
            let challenge = session.challenge.as_deref().unwrap();
            if !verify_authentication_proof(&state.token, challenge, &proof) {
                return Response::error(
                    request.request_id,
                    "authentication_failed",
                    "invalid authentication proof",
                );
            }
            session.authenticated = true;
            Response::new(request.request_id, ResponseResult::Authenticated)
        }
        RequestOperation::Health if !session.authenticated => Response::error(
            request.request_id,
            "authentication_required",
            "authenticate before other requests",
        ),
        RequestOperation::Health => Response::new(
            request.request_id,
            ResponseResult::Health {
                agent_name: state.agent_name.clone(),
                uptime_seconds: state.started_at.elapsed().as_secs(),
            },
        ),
        RequestOperation::DeploymentInventory if !session.authenticated => Response::error(
            request.request_id,
            "authentication_required",
            "authenticate before reading deployment inventory",
        ),
        RequestOperation::DeploymentInventory => match deployment_inventory(state) {
            Ok((deployment_id, files)) => Response::new(
                request.request_id,
                ResponseResult::DeploymentInventory {
                    deployment_id,
                    files,
                },
            ),
            Err(message) => Response::error(request.request_id, "inventory_error", message),
        },
        RequestOperation::PrepareDeployment { .. } if !session.authenticated => Response::error(
            request.request_id,
            "authentication_required",
            "authenticate before preparing a deployment",
        ),
        RequestOperation::PrepareDeployment { manifest, proof } => {
            if session.active_transfer.is_some() {
                return Response::error(
                    request.request_id,
                    "transfer_in_progress",
                    "complete the active file before preparing another deployment",
                );
            }
            if let Err(message) = validate_deployment_manifest(&manifest) {
                return Response::error(request.request_id, "invalid_manifest", message);
            }
            let proof_message = deployment_proof_message(
                session.challenge.as_deref().unwrap(),
                &manifest.deployment_id,
            );
            if !verify_authentication_proof(&state.token, &proof_message, &proof) {
                return Response::error(
                    request.request_id,
                    "authentication_failed",
                    "invalid deployment proof",
                );
            }

            let deployment_id = manifest.deployment_id.clone();
            let required_files = manifest
                .files
                .iter()
                .filter(|file| !staged_file_matches(state, &deployment_id, file))
                .map(|file| file.path.clone())
                .collect();
            session.pending_deployment = Some(manifest);
            Response::new(
                request.request_id,
                ResponseResult::DeploymentPrepared {
                    deployment_id,
                    required_files,
                },
            )
        }
        RequestOperation::BeginFile { .. } if !session.authenticated => Response::error(
            request.request_id,
            "authentication_required",
            "authenticate before transferring files",
        ),
        RequestOperation::BeginFile {
            deployment_id,
            path,
        } => match begin_file(session, state, &deployment_id, &path) {
            Ok(offset) => Response::new(
                request.request_id,
                ResponseResult::FileReady { path, offset },
            ),
            Err((code, message)) => Response::error(request.request_id, code, message),
        },
        RequestOperation::CompleteFile { .. } if !session.authenticated => Response::error(
            request.request_id,
            "authentication_required",
            "authenticate before transferring files",
        ),
        RequestOperation::CompleteFile {
            deployment_id,
            path,
        } => match complete_file(session, state, &deployment_id, &path) {
            Ok(()) => Response::new(request.request_id, ResponseResult::FileCompleted { path }),
            Err((code, message)) => Response::error(request.request_id, code, message),
        },
        RequestOperation::ActivateDeployment { .. } if !session.authenticated => Response::error(
            request.request_id,
            "authentication_required",
            "authenticate before activating a deployment",
        ),
        RequestOperation::ActivateDeployment {
            deployment_id,
            proof,
        } => {
            let proof_message =
                activation_proof_message(session.challenge.as_deref().unwrap(), &deployment_id);
            if !verify_authentication_proof(&state.token, &proof_message, &proof) {
                return Response::error(
                    request.request_id,
                    "authentication_failed",
                    "invalid activation proof",
                );
            }
            match activate_deployment(session, state, &deployment_id) {
                Ok(result) => Response::new(
                    request.request_id,
                    ResponseResult::DeploymentActivated {
                        deployment_id,
                        files: result.files,
                        deletions: result.deletions,
                    },
                ),
                Err((code, message)) => Response::error(request.request_id, code, message),
            }
        }
        RequestOperation::BuildDeployment { .. } => Response::error(
            request.request_id,
            "internal_protocol_error",
            "build requests must use the streaming request handler",
        ),
        RequestOperation::RestartService { .. } if !session.authenticated => Response::error(
            request.request_id,
            "authentication_required",
            "authenticate before restarting the service",
        ),
        RequestOperation::RestartService {
            deployment_id,
            proof,
        } => {
            let proof_message =
                restart_proof_message(session.challenge.as_deref().unwrap(), &deployment_id);
            if !verify_authentication_proof(&state.token, &proof_message, &proof) {
                return Response::error(
                    request.request_id,
                    "authentication_failed",
                    "invalid restart proof",
                );
            }
            match restart_service(state, &deployment_id) {
                Ok(()) => Response::new(
                    request.request_id,
                    ResponseResult::ServiceRestarted {
                        deployment_id,
                        service: state.service_name.clone(),
                    },
                ),
                Err((code, message)) => Response::error(request.request_id, code, message),
            }
        }
    }
}

fn begin_file(
    session: &mut Session,
    state: &ServerState,
    deployment_id: &str,
    path: &str,
) -> Result<u64, (&'static str, String)> {
    if session.active_transfer.is_some() {
        return Err((
            "transfer_in_progress",
            "complete the active file before beginning another".into(),
        ));
    }
    let Some(manifest) = session.pending_deployment.as_ref() else {
        return Err((
            "deployment_not_prepared",
            "prepare a deployment first".into(),
        ));
    };
    if manifest.deployment_id != deployment_id {
        return Err((
            "deployment_mismatch",
            "the requested deployment is not prepared in this session".into(),
        ));
    }
    let Some(file) = manifest
        .files
        .iter()
        .find(|file| file.path == path)
        .cloned()
    else {
        return Err((
            "file_not_in_manifest",
            format!("{path} is not part of the prepared deployment"),
        ));
    };
    if staged_file_matches(state, deployment_id, &file) {
        return Err((
            "file_already_staged",
            format!("{path} is already staged and verified"),
        ));
    }

    let key = format!("{deployment_id}:{path}");
    let mut active_files = state.active_files.lock().unwrap();
    if !active_files.insert(key.clone()) {
        return Err((
            "file_busy",
            format!("another session is transferring {path}"),
        ));
    }
    drop(active_files);

    let partial_path = staging_path(state, deployment_id, "partial", path);
    let final_path = staging_path(state, deployment_id, "files", path);
    let result = (|| -> Result<u64, String> {
        fs::create_dir_all(partial_path.parent().unwrap()).map_err(|error| error.to_string())?;
        let partial = OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(&partial_path)
            .map_err(|error| error.to_string())?;
        let mut offset = partial.metadata().map_err(|error| error.to_string())?.len();
        if offset > file.size {
            partial.set_len(0).map_err(|error| error.to_string())?;
            offset = 0;
        } else if offset == file.size
            && hash_file(&partial_path).is_ok_and(|hash| hash != file.sha256)
        {
            partial.set_len(0).map_err(|error| error.to_string())?;
            offset = 0;
        }
        Ok(offset)
    })();

    match result {
        Ok(offset) => {
            session.active_transfer = Some(ActiveTransfer {
                key,
                file,
                partial_path,
                final_path,
                received: offset,
            });
            Ok(offset)
        }
        Err(message) => {
            state.active_files.lock().unwrap().remove(&key);
            Err(("staging_io_error", message))
        }
    }
}

fn handle_file_chunk(frame: &[u8], session: &mut Session) -> Response {
    let chunk = match decode_file_chunk(frame) {
        Ok(chunk) => chunk,
        Err(message) => return Response::error(0, "invalid_file_chunk", message),
    };
    if !session.authenticated {
        return Response::error(
            chunk.request_id,
            "authentication_required",
            "authenticate before transferring files",
        );
    }
    let Some(transfer) = session.active_transfer.as_mut() else {
        return Response::error(
            chunk.request_id,
            "file_not_started",
            "begin a file before sending chunks",
        );
    };
    if chunk.offset != transfer.received {
        return Response::error(
            chunk.request_id,
            "unexpected_offset",
            format!(
                "expected offset {}, received {}",
                transfer.received, chunk.offset
            ),
        );
    }
    let next_offset = match transfer.received.checked_add(chunk.data.len() as u64) {
        Some(offset) if offset <= transfer.file.size => offset,
        _ => {
            return Response::error(
                chunk.request_id,
                "file_too_large",
                "chunk would exceed the size declared in the manifest",
            );
        }
    };

    let write_result = (|| -> Result<(), std::io::Error> {
        let mut file = OpenOptions::new()
            .write(true)
            .open(&transfer.partial_path)?;
        file.seek(SeekFrom::Start(chunk.offset))?;
        file.write_all(chunk.data)
    })();
    if let Err(error) = write_result {
        return Response::error(chunk.request_id, "staging_io_error", error.to_string());
    }
    transfer.received = next_offset;
    Response::new(
        chunk.request_id,
        ResponseResult::ChunkAccepted {
            path: transfer.file.path.clone(),
            next_offset,
        },
    )
}

fn complete_file(
    session: &mut Session,
    state: &ServerState,
    deployment_id: &str,
    path: &str,
) -> Result<(), (&'static str, String)> {
    let Some(transfer) = session.active_transfer.as_mut() else {
        return Err((
            "file_not_started",
            "begin the file before completing it".into(),
        ));
    };
    let expected_key = format!("{deployment_id}:{path}");
    if transfer.key != expected_key || transfer.file.path != path {
        return Err((
            "file_mismatch",
            "complete the currently active file first".into(),
        ));
    }
    if transfer.received != transfer.file.size {
        return Err((
            "file_incomplete",
            format!(
                "received {} of {} bytes",
                transfer.received, transfer.file.size
            ),
        ));
    }
    let actual_hash = hash_file(&transfer.partial_path)
        .map_err(|error| ("staging_io_error", error.to_string()))?;
    if actual_hash != transfer.file.sha256 {
        File::create(&transfer.partial_path)
            .map_err(|error| ("staging_io_error", error.to_string()))?;
        transfer.received = 0;
        return Err((
            "hash_mismatch",
            format!("SHA-256 verification failed for {path}; partial file was reset"),
        ));
    }

    fs::create_dir_all(transfer.final_path.parent().unwrap())
        .map_err(|error| ("staging_io_error", error.to_string()))?;
    fs::rename(&transfer.partial_path, &transfer.final_path)
        .map_err(|error| ("staging_io_error", error.to_string()))?;
    release_active_transfer(session, state);
    Ok(())
}

fn release_active_transfer(session: &mut Session, state: &ServerState) {
    if let Some(transfer) = session.active_transfer.take() {
        state.active_files.lock().unwrap().remove(&transfer.key);
    }
}

fn staged_file_matches(state: &ServerState, deployment_id: &str, file: &DeploymentFile) -> bool {
    let path = staging_path(state, deployment_id, "files", &file.path);
    fs::metadata(&path).is_ok_and(|metadata| metadata.len() == file.size)
        && hash_file(&path).is_ok_and(|hash| hash == file.sha256)
}

fn staging_path(state: &ServerState, deployment_id: &str, category: &str, path: &str) -> PathBuf {
    state
        .staging_root
        .join(deployment_id)
        .join(category)
        .join(path)
}

fn hash_file(path: &Path) -> Result<String, std::io::Error> {
    let mut reader = BufReader::new(File::open(path)?);
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let count = reader.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        hasher.update(&buffer[..count]);
    }
    Ok(hex::encode(hasher.finalize()))
}

fn handle_build_request(request: Request, session: &Session, state: &ServerState) -> Response {
    let request_id = request.request_id;
    if request.protocol_version != PROTOCOL_VERSION {
        return Response::error(
            request_id,
            "unsupported_protocol",
            format!(
                "agent supports protocol {PROTOCOL_VERSION}, received {}",
                request.protocol_version
            ),
        );
    }
    let RequestOperation::BuildDeployment {
        deployment_id,
        proof,
    } = request.operation
    else {
        unreachable!();
    };
    if !session.authenticated {
        return Response::error(
            request_id,
            "authentication_required",
            "authenticate before building a deployment",
        );
    }
    let proof_message = build_proof_message(session.challenge.as_deref().unwrap(), &deployment_id);
    if !verify_authentication_proof(&state.token, &proof_message, &proof) {
        return Response::error(request_id, "authentication_failed", "invalid build proof");
    }

    let _guard = state.activation_lock.lock().unwrap();
    let current = match load_current_manifest(state) {
        Ok(Some(manifest)) if manifest.deployment_id == deployment_id => manifest,
        Ok(Some(_)) => {
            return Response::error(
                request_id,
                "deployment_mismatch",
                "only the currently activated deployment can be built",
            );
        }
        Ok(None) => {
            return Response::error(
                request_id,
                "deployment_not_activated",
                "activate a deployment before building it",
            );
        }
        Err(message) => return Response::error(request_id, "build_failed", message),
    };
    let workspace = state.deployment_root.join("ros_ws");
    if !workspace.is_dir() {
        return Response::error(
            request_id,
            "build_failed",
            format!("ROS workspace not found: {}", workspace.display()),
        );
    }
    if let Err(message) = clear_build_record(state) {
        return Response::error(request_id, "build_failed", message);
    }

    let script = r#"set -e; . "$1"; shift; exec "$@" build --cmake-args -DBUILD_TESTING=OFF"#;
    let mut child = match Command::new(&state.build_shell)
        .arg("-c")
        .arg(script)
        .arg("pennair-build")
        .arg(&state.ros_setup)
        .arg(&state.colcon)
        .current_dir(&workspace)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
    {
        Ok(child) => child,
        Err(error) => return Response::error(request_id, "build_failed", error.to_string()),
    };

    let (sender, receiver) = mpsc::channel();
    stream_build_output(
        child.stdout.take().unwrap(),
        OutputStream::Stdout,
        sender.clone(),
    );
    stream_build_output(
        child.stderr.take().unwrap(),
        OutputStream::Stderr,
        sender.clone(),
    );
    drop(sender);

    let mut output = Vec::new();
    for (stream, line) in receiver {
        output.push(BuildOutputLine { stream, line });
    }
    let status = match child.wait() {
        Ok(status) => status,
        Err(error) => return Response::error(request_id, "build_failed", error.to_string()),
    };
    if status.success()
        && let Err(message) = save_build_record(state, &current.deployment_id)
    {
        return Response::error(request_id, "build_failed", message);
    }
    Response::new(
        request_id,
        ResponseResult::BuildCompleted {
            deployment_id: current.deployment_id,
            success: status.success(),
            exit_code: status.code(),
            output,
        },
    )
}

fn stream_build_output<R: Read + Send + 'static>(
    reader: R,
    stream: OutputStream,
    sender: mpsc::Sender<(OutputStream, String)>,
) {
    thread::spawn(move || {
        let mut reader = BufReader::new(reader);
        let mut bytes = Vec::new();
        loop {
            bytes.clear();
            match reader.read_until(b'\n', &mut bytes) {
                Ok(0) => break,
                Ok(_) => {
                    while matches!(bytes.last(), Some(b'\n' | b'\r')) {
                        bytes.pop();
                    }
                    if sender
                        .send((stream, String::from_utf8_lossy(&bytes).into_owned()))
                        .is_err()
                    {
                        break;
                    }
                }
                Err(error) => {
                    let _ = sender.send((
                        OutputStream::Stderr,
                        format!("failed to read build output: {error}"),
                    ));
                    break;
                }
            }
        }
    });
}

struct ActivationResult {
    files: usize,
    deletions: usize,
}

fn initialize_deployment_root(root: &Path) -> Result<(), Box<dyn Error>> {
    if fs::symlink_metadata(root).is_ok_and(|metadata| metadata.file_type().is_symlink()) {
        return Err(format!("deployment root cannot be a symlink: {}", root.display()).into());
    }
    fs::create_dir_all(root)?;
    let control = root.join(".pennair");
    if fs::symlink_metadata(&control).is_ok_and(|metadata| metadata.file_type().is_symlink()) {
        return Err(format!(
            "agent control directory cannot be a symlink: {}",
            control.display()
        )
        .into());
    }
    fs::create_dir_all(control.join("staging"))?;
    fs::create_dir_all(control.join("activation"))?;
    fs::create_dir_all(control.join("backups"))?;
    Ok(())
}

fn deployment_inventory(state: &ServerState) -> Result<(Option<String>, Vec<String>), String> {
    let _guard = state.activation_lock.lock().unwrap();
    let Some(manifest) = load_current_manifest(state)? else {
        return Ok((None, Vec::new()));
    };
    Ok((
        Some(manifest.deployment_id),
        manifest.files.into_iter().map(|file| file.path).collect(),
    ))
}

fn activate_deployment(
    session: &Session,
    state: &ServerState,
    deployment_id: &str,
) -> Result<ActivationResult, (&'static str, String)> {
    let Some(manifest) = session.pending_deployment.as_ref() else {
        return Err((
            "deployment_not_prepared",
            "prepare a deployment first".into(),
        ));
    };
    if manifest.deployment_id != deployment_id {
        return Err((
            "deployment_mismatch",
            "the requested deployment is not prepared in this session".into(),
        ));
    }
    let _guard = state.activation_lock.lock().unwrap();
    if load_current_manifest(state)
        .map_err(|message| ("activation_failed", message))?
        .is_some_and(|current| current.deployment_id == deployment_id)
    {
        return Ok(ActivationResult {
            files: manifest.files.len(),
            deletions: manifest.deletions.len(),
        });
    }

    for file in &manifest.files {
        if !staged_file_matches(state, deployment_id, file) {
            return Err((
                "deployment_not_staged",
                format!("{} is missing or failed verification", file.path),
            ));
        }
        ensure_active_path_safe(&state.deployment_root, &file.path)
            .map_err(|message| ("unsafe_active_path", message))?;
    }
    for path in &manifest.deletions {
        ensure_active_path_safe(&state.deployment_root, path)
            .map_err(|message| ("unsafe_active_path", message))?;
    }

    let transaction_id = format!("{}-{}", &deployment_id[..12], &random_challenge()[..12]);
    let activation_root = state
        .deployment_root
        .join(".pennair/activation")
        .join(&transaction_id);
    let backup_root = state
        .deployment_root
        .join(".pennair/backups")
        .join(&transaction_id);

    for file in &manifest.files {
        let source = staging_path(state, deployment_id, "files", &file.path);
        let destination = activation_root.join(&file.path);
        fs::create_dir_all(destination.parent().unwrap())
            .map_err(|error| ("activation_failed", error.to_string()))?;
        fs::copy(&source, &destination)
            .map_err(|error| ("activation_failed", error.to_string()))?;
        if !matches!(hash_file(&destination), Ok(hash) if hash == file.sha256) {
            return Err((
                "activation_failed",
                format!("activation copy failed verification for {}", file.path),
            ));
        }
    }

    let mut installed = Vec::new();
    let mut backed_up = Vec::new();
    let activation = (|| -> Result<(), String> {
        for file in &manifest.files {
            let active = state.deployment_root.join(&file.path);
            if move_to_backup(&active, &backup_root.join(&file.path))? {
                backed_up.push(file.path.clone());
            }
            fs::create_dir_all(active.parent().unwrap()).map_err(|error| error.to_string())?;
            fs::rename(activation_root.join(&file.path), &active)
                .map_err(|error| error.to_string())?;
            installed.push(file.path.clone());
        }
        for path in &manifest.deletions {
            if move_to_backup(&state.deployment_root.join(path), &backup_root.join(path))? {
                backed_up.push(path.clone());
            }
        }
        clear_build_record(state)?;
        save_current_manifest(state, manifest)
    })();

    if let Err(message) = activation {
        let rollback_errors = rollback_activation(state, &backup_root, &installed, &backed_up);
        let suffix = if rollback_errors.is_empty() {
            String::new()
        } else {
            format!("; rollback errors: {}", rollback_errors.join("; "))
        };
        return Err(("activation_failed", format!("{message}{suffix}")));
    }

    let _ = fs::remove_dir_all(&activation_root);
    Ok(ActivationResult {
        files: manifest.files.len(),
        deletions: manifest.deletions.len(),
    })
}

fn ensure_active_path_safe(root: &Path, relative: &str) -> Result<(), String> {
    let components: Vec<_> = Path::new(relative).components().collect();
    let mut current = root.to_path_buf();
    for (index, component) in components.iter().enumerate() {
        current.push(component.as_os_str());
        match fs::symlink_metadata(&current) {
            Ok(metadata) if metadata.file_type().is_symlink() => {
                return Err(format!(
                    "active path contains a symlink: {}",
                    current.display()
                ));
            }
            Ok(metadata) if index + 1 < components.len() && !metadata.is_dir() => {
                return Err(format!(
                    "active path parent is not a directory: {}",
                    current.display()
                ));
            }
            Ok(metadata) if index + 1 == components.len() && !metadata.is_file() => {
                return Err(format!(
                    "active path is not a regular file: {}",
                    current.display()
                ));
            }
            Ok(_) => {}
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => break,
            Err(error) => return Err(error.to_string()),
        }
    }
    Ok(())
}

fn move_to_backup(active: &Path, backup: &Path) -> Result<bool, String> {
    match fs::symlink_metadata(active) {
        Ok(metadata) if metadata.file_type().is_symlink() || !metadata.is_file() => Err(format!(
            "refusing to replace non-regular active path: {}",
            active.display()
        )),
        Ok(_) => {
            fs::create_dir_all(backup.parent().unwrap()).map_err(|error| error.to_string())?;
            fs::rename(active, backup).map_err(|error| error.to_string())?;
            Ok(true)
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(false),
        Err(error) => Err(error.to_string()),
    }
}

fn rollback_activation(
    state: &ServerState,
    backup_root: &Path,
    installed: &[String],
    backed_up: &[String],
) -> Vec<String> {
    let mut errors = Vec::new();
    for path in installed.iter().rev() {
        let active = state.deployment_root.join(path);
        if let Err(error) = fs::remove_file(&active)
            && error.kind() != std::io::ErrorKind::NotFound
        {
            errors.push(format!("remove {}: {error}", active.display()));
        }
    }
    for path in backed_up.iter().rev() {
        let active = state.deployment_root.join(path);
        let backup = backup_root.join(path);
        if let Some(parent) = active.parent()
            && let Err(error) = fs::create_dir_all(parent)
        {
            errors.push(format!("create {}: {error}", parent.display()));
            continue;
        }
        if let Err(error) = fs::rename(&backup, &active) {
            errors.push(format!("restore {}: {error}", active.display()));
        }
    }
    errors
}

fn current_manifest_path(state: &ServerState) -> PathBuf {
    state.deployment_root.join(".pennair/current.json")
}

fn load_current_manifest(state: &ServerState) -> Result<Option<DeploymentManifest>, String> {
    let path = current_manifest_path(state);
    let contents = match fs::read(&path) {
        Ok(contents) => contents,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(error.to_string()),
    };
    let manifest: DeploymentManifest =
        serde_json::from_slice(&contents).map_err(|error| error.to_string())?;
    validate_deployment_manifest(&manifest)?;
    Ok(Some(manifest))
}

fn save_current_manifest(state: &ServerState, manifest: &DeploymentManifest) -> Result<(), String> {
    let path = current_manifest_path(state);
    let partial = path.with_extension("json.partial");
    let bytes = serde_json::to_vec_pretty(manifest).map_err(|error| error.to_string())?;
    let mut file = File::create(&partial).map_err(|error| error.to_string())?;
    file.write_all(&bytes).map_err(|error| error.to_string())?;
    file.sync_all().map_err(|error| error.to_string())?;
    fs::rename(partial, path).map_err(|error| error.to_string())
}

fn build_record_path(state: &ServerState) -> PathBuf {
    state.deployment_root.join(".pennair/built")
}

fn clear_build_record(state: &ServerState) -> Result<(), String> {
    match fs::remove_file(build_record_path(state)) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error.to_string()),
    }
}

fn save_build_record(state: &ServerState, deployment_id: &str) -> Result<(), String> {
    let path = build_record_path(state);
    let partial = path.with_extension("partial");
    let mut file = File::create(&partial).map_err(|error| error.to_string())?;
    file.write_all(deployment_id.as_bytes())
        .map_err(|error| error.to_string())?;
    file.write_all(b"\n").map_err(|error| error.to_string())?;
    file.sync_all().map_err(|error| error.to_string())?;
    fs::rename(partial, path).map_err(|error| error.to_string())
}

fn restart_service(state: &ServerState, deployment_id: &str) -> Result<(), (&'static str, String)> {
    let _guard = state.activation_lock.lock().unwrap();
    let current = load_current_manifest(state)
        .map_err(|message| ("restart_failed", message))?
        .ok_or_else(|| {
            (
                "deployment_not_activated",
                "activate and build a deployment before restarting".into(),
            )
        })?;
    if current.deployment_id != deployment_id {
        return Err((
            "deployment_mismatch",
            "only the currently activated deployment can be restarted".into(),
        ));
    }
    let built = fs::read_to_string(build_record_path(state)).map_err(|error| {
        if error.kind() == std::io::ErrorKind::NotFound {
            (
                "build_required",
                "the active deployment has not completed a successful build".into(),
            )
        } else {
            ("restart_failed", error.to_string())
        }
    })?;
    if built.trim() != deployment_id {
        return Err((
            "build_required",
            "the successful build does not match the active deployment".into(),
        ));
    }

    let output = Command::new(&state.sudo)
        .arg("-n")
        .arg(&state.systemctl)
        .arg("restart")
        .arg(&state.service_name)
        .output()
        .map_err(|error| ("restart_failed", error.to_string()))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let stdout = String::from_utf8_lossy(&output.stdout);
        let message = if !stderr.trim().is_empty() {
            stderr.trim().to_owned()
        } else if !stdout.trim().is_empty() {
            stdout.trim().to_owned()
        } else {
            format!("restart command failed with {}", output.status)
        };
        return Err(("restart_failed", message));
    }
    Ok(())
}

fn random_challenge() -> String {
    let mut bytes = [0_u8; 32];
    rand::rng().fill_bytes(&mut bytes);
    hex::encode(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pennair_protocol::{
        DeploymentFile, authentication_proof, calculate_deployment_id, deployment_proof_message,
    };

    fn unique_test_root(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "pennair-agent-{name}-{}-{}",
            std::process::id(),
            &random_challenge()[..12]
        ))
    }

    fn state_at(deployment_root: PathBuf) -> ServerState {
        ServerState {
            agent_name: "air-test".into(),
            token: "a test token that contains at least 32 characters".into(),
            staging_root: deployment_root.join(".pennair/staging"),
            deployment_root,
            build_shell: PathBuf::from("/bin/bash"),
            ros_setup: PathBuf::from("/dev/null"),
            colcon: PathBuf::from("/usr/bin/true"),
            sudo: PathBuf::from("/usr/bin/true"),
            systemctl: PathBuf::from("/usr/bin/true"),
            service_name: "pennair-autonomy.service".into(),
            active_files: Mutex::new(HashSet::new()),
            activation_lock: Mutex::new(()),
            sessions: Mutex::new(HashMap::new()),
            started_at: Instant::now(),
        }
    }

    fn state() -> ServerState {
        state_at(std::env::temp_dir().join("pennair-agent-tests"))
    }

    fn deployment_file(path: &str, contents: &[u8]) -> DeploymentFile {
        DeploymentFile {
            path: path.into(),
            size: contents.len() as u64,
            sha256: hex::encode(Sha256::digest(contents)),
        }
    }

    #[test]
    fn health_requires_authentication() {
        let response = dispatch(
            Request::new(1, RequestOperation::Health),
            &mut Session::default(),
            &state(),
        );

        assert!(matches!(
            response.result,
            ResponseResult::Error { ref code, .. } if code == "authentication_required"
        ));
    }

    #[test]
    fn authenticated_session_enables_health() {
        let mut session = Session::default();
        let server = state();
        let hello = dispatch(
            Request::new(
                1,
                RequestOperation::Hello {
                    client_name: "test-client".into(),
                },
            ),
            &mut session,
            &server,
        );
        let ResponseResult::Hello { auth_challenge, .. } = hello.result else {
            panic!("expected hello response");
        };
        let proof = pennair_protocol::authentication_proof(&server.token, &auth_challenge);
        let authenticated = dispatch(
            Request::new(2, RequestOperation::Authenticate { proof }),
            &mut session,
            &server,
        );
        assert_eq!(authenticated.result, ResponseResult::Authenticated);

        let response = dispatch(
            Request::new(3, RequestOperation::Health),
            &mut session,
            &server,
        );

        assert!(matches!(
            response.result,
            ResponseResult::Health { ref agent_name, .. } if agent_name == "air-test"
        ));
    }

    #[test]
    fn authenticated_session_prepares_valid_manifest() {
        let mut session = Session::default();
        let server = state();
        let hello = dispatch(
            Request::new(
                1,
                RequestOperation::Hello {
                    client_name: "test-client".into(),
                },
            ),
            &mut session,
            &server,
        );
        let ResponseResult::Hello { auth_challenge, .. } = hello.result else {
            panic!("expected hello response");
        };
        let auth_proof = authentication_proof(&server.token, &auth_challenge);
        dispatch(
            Request::new(2, RequestOperation::Authenticate { proof: auth_proof }),
            &mut session,
            &server,
        );

        let files = vec![DeploymentFile {
            path: "ros_ws/src/package/file.py".into(),
            size: 12,
            sha256: "a".repeat(64),
        }];
        let manifest = DeploymentManifest {
            deployment_id: calculate_deployment_id(&files, &[], false),
            files,
            deletions: vec![],
            restart: false,
        };
        let proof = authentication_proof(
            &server.token,
            &deployment_proof_message(&auth_challenge, &manifest.deployment_id),
        );
        let response = dispatch(
            Request::new(3, RequestOperation::PrepareDeployment { manifest, proof }),
            &mut session,
            &server,
        );

        assert!(matches!(
            response.result,
            ResponseResult::DeploymentPrepared { ref required_files, .. }
                if required_files == &["ros_ws/src/package/file.py"]
        ));
        assert!(session.pending_deployment.is_some());
    }

    #[test]
    fn activation_replaces_files_and_applies_explicit_deletions() {
        let root = unique_test_root("activation");
        initialize_deployment_root(&root).unwrap();
        let state = state_at(root.clone());
        fs::create_dir_all(root.join("pkg")).unwrap();
        fs::write(root.join("pkg/replace.txt"), b"old").unwrap();
        fs::write(root.join("pkg/stale.txt"), b"stale").unwrap();

        let old_files = vec![
            deployment_file("pkg/replace.txt", b"old"),
            deployment_file("pkg/stale.txt", b"stale"),
        ];
        let old_manifest = DeploymentManifest {
            deployment_id: calculate_deployment_id(&old_files, &[], false),
            files: old_files,
            deletions: vec![],
            restart: false,
        };
        save_current_manifest(&state, &old_manifest).unwrap();
        save_build_record(&state, &old_manifest.deployment_id).unwrap();

        let files = vec![deployment_file("pkg/replace.txt", b"new")];
        let deletions = vec!["pkg/stale.txt".into()];
        let manifest = DeploymentManifest {
            deployment_id: calculate_deployment_id(&files, &deletions, false),
            files,
            deletions,
            restart: false,
        };
        let staged = staging_path(&state, &manifest.deployment_id, "files", "pkg/replace.txt");
        fs::create_dir_all(staged.parent().unwrap()).unwrap();
        fs::write(staged, b"new").unwrap();
        let session = Session {
            authenticated: true,
            pending_deployment: Some(manifest.clone()),
            ..Session::default()
        };

        let result = activate_deployment(&session, &state, &manifest.deployment_id).unwrap();

        assert_eq!(result.files, 1);
        assert_eq!(result.deletions, 1);
        assert_eq!(fs::read(root.join("pkg/replace.txt")).unwrap(), b"new");
        assert!(!root.join("pkg/stale.txt").exists());
        assert_eq!(load_current_manifest(&state).unwrap().unwrap(), manifest);
        assert!(!build_record_path(&state).exists());
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn activation_rolls_back_when_current_record_cannot_be_written() {
        let root = unique_test_root("rollback");
        initialize_deployment_root(&root).unwrap();
        let state = state_at(root.clone());
        fs::create_dir_all(root.join("pkg")).unwrap();
        fs::write(root.join("pkg/replace.txt"), b"old").unwrap();

        let files = vec![deployment_file("pkg/replace.txt", b"new")];
        let manifest = DeploymentManifest {
            deployment_id: calculate_deployment_id(&files, &[], false),
            files,
            deletions: vec![],
            restart: false,
        };
        let staged = staging_path(&state, &manifest.deployment_id, "files", "pkg/replace.txt");
        fs::create_dir_all(staged.parent().unwrap()).unwrap();
        fs::write(staged, b"new").unwrap();
        fs::create_dir_all(root.join(".pennair/current.json.partial")).unwrap();
        let session = Session {
            authenticated: true,
            pending_deployment: Some(manifest.clone()),
            ..Session::default()
        };

        let result = activate_deployment(&session, &state, &manifest.deployment_id);

        assert!(matches!(result, Err(("activation_failed", _))));
        assert_eq!(fs::read(root.join("pkg/replace.txt")).unwrap(), b"old");
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn restart_requires_matching_successful_build() {
        let root = unique_test_root("restart");
        initialize_deployment_root(&root).unwrap();
        let state = state_at(root.clone());
        let manifest = DeploymentManifest {
            deployment_id: calculate_deployment_id(&[], &[], false),
            files: vec![],
            deletions: vec![],
            restart: false,
        };
        save_current_manifest(&state, &manifest).unwrap();

        assert!(matches!(
            restart_service(&state, &manifest.deployment_id),
            Err(("build_required", _))
        ));

        save_build_record(&state, &manifest.deployment_id).unwrap();
        assert_eq!(restart_service(&state, &manifest.deployment_id), Ok(()));
        fs::remove_dir_all(root).unwrap();
    }
}
