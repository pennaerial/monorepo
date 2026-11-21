use dioxus::html::FileData;
use pennair_protocol::{
    activation_proof_message, authentication_proof, build_proof_message, calculate_deployment_id,
    deployment_proof_message, encode_file_chunk, foxglove_service_for_operation,
    restart_proof_message, validate_deployment_manifest, BuildOutputLine, DeploymentFile,
    DeploymentManifest, Request, RequestOperation, Response, ResponseResult, FOXGLOVE_SERVICE_TYPE,
    MAX_FILE_CHUNK_SIZE, PROTOCOL_VERSION, SERVICE_WRITE_CHUNK,
};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    time::{Duration, Instant},
};

use crate::ros::{NodeError, NodeHandle};

const SERVICE_DISCOVERY_TIMEOUT: Duration = Duration::from_secs(5);
const REQUEST_TIMEOUT: Duration = Duration::from_secs(30);
const BUILD_TIMEOUT: Duration = Duration::from_secs(30 * 60);

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DeploymentProgress {
    Prepared {
        files: usize,
        required: usize,
    },
    FileStaged {
        path: String,
        completed: usize,
        total: usize,
    },
    Activated {
        files: usize,
        deletions: usize,
    },
    BuildOutput(BuildOutputLine),
    Restarted {
        service: String,
    },
    Complete {
        deployment_id: String,
    },
}

pub struct DeploymentClient {
    node: NodeHandle,
    token: String,
    challenge: String,
    next_request_id: u64,
}

impl DeploymentClient {
    pub async fn authenticate(node: NodeHandle, token: String) -> Result<Self, String> {
        if token.len() < 32 {
            return Err("deployment token must contain at least 32 characters".into());
        }
        let mut client = Self {
            node,
            token,
            challenge: String::new(),
            next_request_id: 1,
        };
        match client
            .request(RequestOperation::Hello {
                client_name: "pennair-gcs".into(),
            })
            .await?
        {
            ResponseResult::Hello { auth_challenge, .. } => {
                client.challenge = auth_challenge;
            }
            result => return Err(unexpected("hello", result)),
        }
        let proof = authentication_proof(&client.token, &client.challenge);
        match client
            .request(RequestOperation::Authenticate { proof })
            .await?
        {
            ResponseResult::Authenticated => Ok(client),
            result => Err(unexpected("authentication", result)),
        }
    }

    pub async fn health(&mut self) -> Result<(String, u64), String> {
        match self.request(RequestOperation::Health).await? {
            ResponseResult::Health {
                agent_name,
                uptime_seconds,
            } => Ok((agent_name, uptime_seconds)),
            result => Err(unexpected("health", result)),
        }
    }

    pub async fn deploy(
        &mut self,
        deployment: &mut BrowserDeployment,
        mut on_progress: impl FnMut(DeploymentProgress),
    ) -> Result<(), String> {
        let (_, active_files) = self.inventory().await?;
        deployment.reconcile_inventory(active_files)?;
        let required_files = self.prepare(deployment.manifest.clone()).await?;
        on_progress(DeploymentProgress::Prepared {
            files: deployment.manifest.files.len(),
            required: required_files.len(),
        });

        let required: HashSet<_> = required_files.into_iter().collect();
        let total = required.len();
        let mut completed = 0;
        for file in &deployment.manifest.files {
            if !required.contains(&file.path) {
                continue;
            }
            let contents = deployment
                .files
                .get(&file.path)
                .ok_or_else(|| format!("selected folder no longer contains {}", file.path))?;
            self.transfer_file(&deployment.manifest.deployment_id, file, contents)
                .await?;
            completed += 1;
            on_progress(DeploymentProgress::FileStaged {
                path: file.path.clone(),
                completed,
                total,
            });
        }

        let (files, deletions) = self.activate(&deployment.manifest.deployment_id).await?;
        on_progress(DeploymentProgress::Activated { files, deletions });
        let (success, exit_code, output) = self.build(&deployment.manifest.deployment_id).await?;
        for line in output {
            on_progress(DeploymentProgress::BuildOutput(line));
        }
        if !success {
            return Err(match exit_code {
                Some(code) => format!("build failed with exit code {code}"),
                None => "build terminated without an exit code".into(),
            });
        }
        if deployment.manifest.restart {
            let service = self.restart(&deployment.manifest.deployment_id).await?;
            on_progress(DeploymentProgress::Restarted { service });
        }
        on_progress(DeploymentProgress::Complete {
            deployment_id: deployment.manifest.deployment_id.clone(),
        });
        Ok(())
    }

    async fn inventory(&mut self) -> Result<(Option<String>, Vec<String>), String> {
        match self.request(RequestOperation::DeploymentInventory).await? {
            ResponseResult::DeploymentInventory {
                deployment_id,
                files,
            } => Ok((deployment_id, files)),
            result => Err(unexpected("inventory", result)),
        }
    }

    async fn prepare(&mut self, manifest: DeploymentManifest) -> Result<Vec<String>, String> {
        let proof = authentication_proof(
            &self.token,
            &deployment_proof_message(&self.challenge, &manifest.deployment_id),
        );
        match self
            .request(RequestOperation::PrepareDeployment { manifest, proof })
            .await?
        {
            ResponseResult::DeploymentPrepared { required_files, .. } => Ok(required_files),
            result => Err(unexpected("prepare", result)),
        }
    }

    async fn transfer_file(
        &mut self,
        deployment_id: &str,
        file: &DeploymentFile,
        contents: &[u8],
    ) -> Result<(), String> {
        let mut offset = match self
            .request(RequestOperation::BeginFile {
                deployment_id: deployment_id.into(),
                path: file.path.clone(),
            })
            .await?
        {
            ResponseResult::FileReady { path, offset } if path == file.path => offset,
            result => return Err(unexpected("begin file", result)),
        };
        if offset > contents.len() as u64 {
            return Err(format!(
                "agent returned an invalid offset for {}",
                file.path
            ));
        }
        while offset < contents.len() as u64 {
            let end = (offset as usize + MAX_FILE_CHUNK_SIZE).min(contents.len());
            let request_id = self.next_id();
            let frame = encode_file_chunk(request_id, offset, &contents[offset as usize..end])?;
            let response = self
                .call_service(SERVICE_WRITE_CHUNK, frame, request_id, REQUEST_TIMEOUT)
                .await?;
            offset = match response.result {
                ResponseResult::ChunkAccepted { path, next_offset }
                    if path == file.path && next_offset == end as u64 =>
                {
                    next_offset
                }
                result => return Err(unexpected("file chunk", result)),
            };
        }
        match self
            .request(RequestOperation::CompleteFile {
                deployment_id: deployment_id.into(),
                path: file.path.clone(),
            })
            .await?
        {
            ResponseResult::FileCompleted { path } if path == file.path => Ok(()),
            result => Err(unexpected("complete file", result)),
        }
    }

    async fn activate(&mut self, deployment_id: &str) -> Result<(usize, usize), String> {
        let proof = authentication_proof(
            &self.token,
            &activation_proof_message(&self.challenge, deployment_id),
        );
        match self
            .request(RequestOperation::ActivateDeployment {
                deployment_id: deployment_id.into(),
                proof,
            })
            .await?
        {
            ResponseResult::DeploymentActivated {
                deployment_id: response_id,
                files,
                deletions,
            } if response_id == deployment_id => Ok((files, deletions)),
            result => Err(unexpected("activate", result)),
        }
    }

    async fn build(
        &mut self,
        deployment_id: &str,
    ) -> Result<(bool, Option<i32>, Vec<BuildOutputLine>), String> {
        let proof = authentication_proof(
            &self.token,
            &build_proof_message(&self.challenge, deployment_id),
        );
        match self
            .request_with_timeout(
                RequestOperation::BuildDeployment {
                    deployment_id: deployment_id.into(),
                    proof,
                },
                BUILD_TIMEOUT,
            )
            .await?
        {
            ResponseResult::BuildCompleted {
                deployment_id: response_id,
                success,
                exit_code,
                output,
            } if response_id == deployment_id => Ok((success, exit_code, output)),
            result => Err(unexpected("build", result)),
        }
    }

    async fn restart(&mut self, deployment_id: &str) -> Result<String, String> {
        let proof = authentication_proof(
            &self.token,
            &restart_proof_message(&self.challenge, deployment_id),
        );
        match self
            .request(RequestOperation::RestartService {
                deployment_id: deployment_id.into(),
                proof,
            })
            .await?
        {
            ResponseResult::ServiceRestarted {
                deployment_id: response_id,
                service,
            } if response_id == deployment_id => Ok(service),
            result => Err(unexpected("restart", result)),
        }
    }

    async fn request(&mut self, operation: RequestOperation) -> Result<ResponseResult, String> {
        self.request_with_timeout(operation, REQUEST_TIMEOUT).await
    }

    async fn request_with_timeout(
        &mut self,
        operation: RequestOperation,
        timeout: Duration,
    ) -> Result<ResponseResult, String> {
        let service = foxglove_service_for_operation(&operation);
        let request_id = self.next_id();
        let payload = serde_json::to_vec(&Request::new(request_id, operation))
            .map_err(|error| error.to_string())?;
        let response = self
            .call_service(service, payload, request_id, timeout)
            .await?;
        match response.result {
            ResponseResult::Error { code, message } => Err(format!("{code}: {message}")),
            result => Ok(result),
        }
    }

    async fn call_service(
        &self,
        service: &str,
        payload: Vec<u8>,
        request_id: u64,
        timeout: Duration,
    ) -> Result<Response, String> {
        let started = Instant::now();
        loop {
            match self
                .node
                .call_raw_service(
                    service,
                    Some(FOXGLOVE_SERVICE_TYPE),
                    payload.clone(),
                    timeout,
                )
                .await
            {
                Ok(bytes) => {
                    let response: Response =
                        serde_json::from_slice(&bytes).map_err(|error| error.to_string())?;
                    if response.protocol_version != PROTOCOL_VERSION {
                        return Err(format!(
                            "agent returned protocol {}, expected {PROTOCOL_VERSION}",
                            response.protocol_version
                        ));
                    }
                    if response.request_id != request_id {
                        return Err(format!(
                            "agent returned request {}, expected {request_id}",
                            response.request_id
                        ));
                    }
                    return Ok(response);
                }
                Err(NodeError::ServiceNotAdvertised(_))
                    if started.elapsed() < SERVICE_DISCOVERY_TIMEOUT =>
                {
                    deployment_sleep(Duration::from_millis(20)).await;
                }
                Err(error) => return Err(error.to_string()),
            }
        }
    }

    fn next_id(&mut self) -> u64 {
        let id = self.next_request_id;
        self.next_request_id = self.next_request_id.wrapping_add(1);
        id
    }
}

pub struct BrowserDeployment {
    pub manifest: DeploymentManifest,
    files: BTreeMap<String, Vec<u8>>,
}

impl BrowserDeployment {
    pub async fn from_selected_folder(files: Vec<FileData>, restart: bool) -> Result<Self, String> {
        let mut named_files = Vec::with_capacity(files.len());
        for file in files {
            let path = file.path().to_string_lossy().replace('\\', "/");
            let bytes = file
                .read_bytes()
                .await
                .map_err(|error| format!("failed to read {path}: {error}"))?;
            named_files.push((path, bytes.to_vec()));
        }
        Self::from_named_files(named_files, restart)
    }

    pub fn from_named_files(
        named_files: Vec<(String, Vec<u8>)>,
        restart: bool,
    ) -> Result<Self, String> {
        let manifest_path = named_files
            .iter()
            .map(|(path, _)| path.replace('\\', "/"))
            .filter(|path| path == "monorepo.toml" || path.ends_with("/monorepo.toml"))
            .min_by_key(|path| path.matches('/').count())
            .ok_or_else(|| "selected folder does not contain monorepo.toml".to_owned())?;
        let root_prefix = manifest_path
            .strip_suffix("monorepo.toml")
            .unwrap()
            .to_owned();
        let manifest_bytes = named_files
            .iter()
            .find(|(path, _)| path.replace('\\', "/") == manifest_path)
            .map(|(_, bytes)| bytes)
            .unwrap();
        let config: BrowserConfig = toml::from_slice(manifest_bytes)
            .map_err(|error| format!("invalid monorepo.toml: {error}"))?;
        let deploy_paths: BTreeSet<String> = config
            .deploy
            .paths
            .into_iter()
            .map(|path| path.trim_matches('/').to_owned())
            .collect();
        if deploy_paths.is_empty() {
            return Err("monorepo.toml [deploy] paths cannot be empty".into());
        }

        let mut selected = BTreeMap::new();
        for (path, bytes) in named_files {
            let path = path.replace('\\', "/");
            let Some(relative) = path.strip_prefix(&root_prefix) else {
                continue;
            };
            if relative
                .split('/')
                .any(|part| part == ".git" || part == "__pycache__")
            {
                continue;
            }
            if deploy_paths
                .iter()
                .any(|deploy| relative == deploy || relative.starts_with(&format!("{deploy}/")))
            {
                selected.insert(relative.to_owned(), bytes);
            }
        }
        for deploy in &deploy_paths {
            if !selected
                .keys()
                .any(|path| path == deploy || path.starts_with(&format!("{deploy}/")))
            {
                return Err(format!(
                    "[deploy] path contains no selected files: {deploy}; select the repository root and initialize required submodules"
                ));
            }
        }

        let files: Vec<DeploymentFile> = selected
            .iter()
            .map(|(path, bytes)| DeploymentFile {
                path: path.clone(),
                size: bytes.len() as u64,
                sha256: hex::encode(Sha256::digest(bytes)),
            })
            .collect();
        let deployment_id = calculate_deployment_id(&files, &[], restart);
        let manifest = DeploymentManifest {
            deployment_id,
            files,
            deletions: Vec::new(),
            restart,
        };
        validate_deployment_manifest(&manifest)?;
        Ok(Self {
            manifest,
            files: selected,
        })
    }

    fn reconcile_inventory(&mut self, active_files: Vec<String>) -> Result<(), String> {
        let local: BTreeSet<&str> = self
            .manifest
            .files
            .iter()
            .map(|file| file.path.as_str())
            .collect();
        self.manifest.deletions = active_files
            .into_iter()
            .filter(|path| !local.contains(path.as_str()))
            .collect();
        self.manifest.deletions.sort();
        self.manifest.deletions.dedup();
        self.manifest.deployment_id = calculate_deployment_id(
            &self.manifest.files,
            &self.manifest.deletions,
            self.manifest.restart,
        );
        validate_deployment_manifest(&self.manifest)
    }
}

#[derive(Deserialize)]
struct BrowserConfig {
    deploy: BrowserDeployConfig,
}

#[derive(Deserialize)]
struct BrowserDeployConfig {
    paths: Vec<String>,
}

fn unexpected(operation: &str, result: ResponseResult) -> String {
    format!("unexpected {operation} response: {result:?}")
}

#[cfg(not(target_arch = "wasm32"))]
async fn deployment_sleep(duration: Duration) {
    tokio::time::sleep(duration).await;
}

#[cfg(target_arch = "wasm32")]
async fn deployment_sleep(duration: Duration) {
    gloo_timers::future::TimeoutFuture::new(duration.as_millis() as u32).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selected_folder_uses_only_configured_deploy_paths() {
        let config = br#"[deploy]
paths = ["monorepo.toml", "ros_ws/src/package"]
"#;
        let deployment = BrowserDeployment::from_named_files(
            vec![
                ("repo/monorepo.toml".into(), config.to_vec()),
                ("repo/ros_ws/src/package/node.py".into(), b"node".to_vec()),
                ("repo/README.md".into(), b"ignored".to_vec()),
            ],
            true,
        )
        .unwrap();

        assert_eq!(
            deployment.files.keys().cloned().collect::<Vec<_>>(),
            ["monorepo.toml", "ros_ws/src/package/node.py"]
        );
        assert!(deployment.manifest.restart);
        assert_eq!(validate_deployment_manifest(&deployment.manifest), Ok(()));
    }

    #[test]
    fn selected_folder_rejects_missing_configured_subtree() {
        let config = br#"[deploy]
paths = ["missing-submodule"]
"#;
        let error = BrowserDeployment::from_named_files(
            vec![("repo/monorepo.toml".into(), config.to_vec())],
            false,
        )
        .err()
        .unwrap();

        assert!(error.contains("missing-submodule"));
    }
}
