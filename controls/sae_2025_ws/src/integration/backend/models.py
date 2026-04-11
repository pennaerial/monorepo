from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class ErrorResponse(BaseModel):
    success: Literal[False] = False
    error: str


class MessageResponse(BaseModel):
    success: Literal[True] = True
    output: str


class OperatorConfigPayload(BaseModel):
    github_repo: str
    github_token: str
    hotspot_name: str
    inventory_path: str
    default_deploy_root: str


class WorkspacePathsPayload(BaseModel):
    deploy_root: str
    current_release: str
    runtime_fleet: str
    fleet_file: str
    overlay_file: str
    missions_dir: str


class TargetPayload(BaseModel):
    target_id: str
    label: str
    pi_user: str
    pi_host: str
    deploy_root: str
    ssh_key: str
    ssh_pass: str
    fleet_file: str
    vehicle_name: str
    overlay_yaml: str
    service_unit: str
    enabled: bool
    ssh_target: str
    workspace_paths: WorkspacePathsPayload


class ConfigResponse(BaseModel):
    success: Literal[True] = True
    config: OperatorConfigPayload
    active_target_id: str
    workspace_paths: WorkspacePathsPayload


class InventoryResponse(BaseModel):
    success: Literal[True] = True
    active_target_id: str
    targets: list[TargetPayload] = Field(default_factory=list)


class InventoryMutationResponse(BaseModel):
    success: Literal[True] = True
    output: str
    target: TargetPayload | None = None
    active_target_id: str | None = None


class ConnectionStatusResponse(BaseModel):
    success: Literal[True] = True
    connected: bool
    target: str | None = None
    info: str = ""
    error: str | None = None


class SSHCommandResponse(BaseModel):
    success: Literal[True] = True
    command: str


class WifiConnection(BaseModel):
    name: str
    type: str
    device: str
    state: str


class WifiStatusResponse(BaseModel):
    success: bool
    is_hotspot: bool | None = None
    current_wifi: str | None = None
    connections: list[WifiConnection] = Field(default_factory=list)
    error: str | None = None


class WifiNetwork(BaseModel):
    ssid: str
    signal: int
    security: str


class WifiScanResponse(BaseModel):
    success: bool
    networks: list[WifiNetwork] = Field(default_factory=list)
    error: str | None = None


class BuildCurrentResponse(BaseModel):
    success: Literal[True] = True
    installed: bool
    info: str
    release_id: str | None = None


class BuildListItem(BaseModel):
    source: Literal["release", "actions"]
    tag: str
    sha: str
    name: str
    date: str
    download_url: str | None = None
    size_mb: float | None = None
    run_id: str | None = None
    artifact_id: str | None = None
    artifact_name: str | None = None
    branch: str | None = None


class BuildListResponse(BaseModel):
    success: bool
    builds: list[BuildListItem] = Field(default_factory=list)
    error: str | None = None


class MissionRuntimeState(BaseModel):
    phase: Literal["idle", "preparing", "running", "stopping", "error", "offline"]
    launch_state: Literal["running", "stopped", "not_prepared", "error", "offline"]
    running: bool
    pid: str | None = None
    message: str | None = None
    error: str | None = None
    updated_at: float


class MissionStateResponse(BaseModel):
    success: Literal[True] = True
    state: MissionRuntimeState


class MissionLaunchStatusResponse(BaseModel):
    success: bool
    running: bool
    state: str
    pid: str | None = None
    error: str | None = None


class MissionLogsResponse(BaseModel):
    success: bool
    running: bool
    logs: str = ""
    next_offset: int | None = None
    inode: int | None = None
    reset: bool | None = None
    error: str | None = None


class LaunchParamsResponse(BaseModel):
    success: bool
    content: str | None = None
    output: str | None = None
    error: str | None = None


class MissionNameOptionsResponse(BaseModel):
    success: bool
    missions: list[str] = Field(default_factory=list)
    error: str | None = None


class MissionFileResponse(BaseModel):
    success: bool
    mission: str | None = None
    path: str | None = None
    content: str | None = None
    output: str | None = None
    error: str | None = None


class TerminalChunkMessage(BaseModel):
    type: Literal["chunk"] = "chunk"
    data: str
    next_offset: int
    inode: int
    reset: bool = False


class TerminalInfoMessage(BaseModel):
    type: Literal["info", "error"]
    message: str


class BuildTransferResult(BaseModel):
    artifact_name: str
    output: str


class SchemaFieldResponse(BaseModel):
    name: str
    schema_type: str
    annotation: str
    required: bool
    default: Any | None = None
    default_kind: Literal["missing", "none", "value", "nan"] = "missing"
    nullable: bool = False
    choices: list[Any] = Field(default_factory=list)
    description: str | None = None
    editable: bool = True
    derived: bool = False
    applies_to: list[str] = Field(default_factory=list)


class ModeMetadataResponse(BaseModel):
    name: str
    class_path: str
    module: str
    mission_target: str
    description: str | None = None
    required_vision_nodes: list[str] = Field(default_factory=list)
    transition_labels: list[str] = Field(default_factory=list)
    params: list[SchemaFieldResponse] = Field(default_factory=list)
    params_schema: dict[str, Any] = Field(default_factory=dict)


class MissionModeResponse(BaseModel):
    name: str
    class_path: str
    params: dict[str, Any] = Field(default_factory=dict)
    transitions: dict[str, str] = Field(default_factory=dict)
    metadata: ModeMetadataResponse


class MissionSchemaResponse(BaseModel):
    success: Literal[True] = True
    name: str
    path: str
    target: str
    start_mode: str
    vision_nodes: list[str] = Field(default_factory=list)
    modes: list[MissionModeResponse] = Field(default_factory=list)
    document_schema: dict[str, Any] = Field(default_factory=dict)


class MissionCatalogResponse(BaseModel):
    success: Literal[True] = True
    available_missions: list[str] = Field(default_factory=list)
    missions: list[MissionSchemaResponse] = Field(default_factory=list)


class ModeRegistryResponse(BaseModel):
    success: Literal[True] = True
    targets: dict[str, list[ModeMetadataResponse]] = Field(default_factory=dict)


class FleetSectionSchemaResponse(BaseModel):
    name: str
    description: str | None = None
    applies_to: list[str] = Field(default_factory=list)
    constraints: list[str] = Field(default_factory=list)
    fields: list[SchemaFieldResponse] = Field(default_factory=list)


class FleetSchemaResponse(BaseModel):
    success: Literal[True] = True
    available_fleets: list[str] = Field(default_factory=list)
    backend_kinds: list[str] = Field(default_factory=list)
    sections: list[FleetSectionSchemaResponse] = Field(default_factory=list)
    excluded_keys: list[str] = Field(default_factory=list)
    document_schema: dict[str, Any] = Field(default_factory=dict)


class SchemaIndexResponse(BaseModel):
    success: Literal[True] = True
    missions: MissionCatalogResponse
    fleet: FleetSchemaResponse
    modes: ModeRegistryResponse
