from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator


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
    default_pi_user: str
    default_ssh_key: str
    default_ssh_pass: str


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
    vehicle_name: str
    overlay_yaml: str
    service_unit: str
    enabled: bool
    ssh_target: str
    workspace_paths: WorkspacePathsPayload


class ConfigResponse(BaseModel):
    success: Literal[True] = True
    config: OperatorConfigPayload


class LiveHardwareDeviceResponse(BaseModel):
    hardware_id: str
    hostname: str
    addresses: list[str] = Field(default_factory=list)
    service_name: str | None = None
    service_type: str | None = None
    last_seen_at: str | None = None
    discovery_stale: bool = False
    matched_target_id: str | None = None
    matched_label: str | None = None
    saved: bool = False


class HardwareDiscoveryResponse(BaseModel):
    success: bool
    devices: list[LiveHardwareDeviceResponse] = Field(default_factory=list)
    error: str | None = None


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
    current_mode: str | None = None
    effective_role: str | None = None
    policy_source: str | None = None
    runtime_active: bool | None = None
    mission_started: bool | None = None
    travel_router_locked: bool | None = None
    local_ap_profile: str | None = None
    travel_router_profile: str | None = None
    allowed_ap_hosts: list[str] = Field(default_factory=list)
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
    commit_sha: str | None = None
    commit_subject: str | None = None
    name: str
    date: str
    download_url: str | None = None
    size_mb: float | None = None
    run_id: str | None = None
    artifact_id: str | None = None
    artifact_name: str | None = None
    branch: str | None = None
    workflow_name: str | None = None
    workflow_event: str | None = None
    workflow_conclusion: str | None = None
    deployable: bool = True
    fallback: bool = False


class BuildListResponse(BaseModel):
    success: bool
    builds: list[BuildListItem] = Field(default_factory=list)
    releases: list[BuildListItem] = Field(default_factory=list)
    artifacts: list[BuildListItem] = Field(default_factory=list)
    artifact_page: int = 1
    artifact_page_size: int = 20
    artifact_has_more: bool = False
    error: str | None = None


NetworkRole = Literal["default", "ap", "client"]
ApSelection = Literal["ordered_then_strongest", "strongest"]


def _normalize_host_alias(value: object) -> str:
    candidate = str(value or "").strip().rstrip(".").lower()
    if candidate.endswith(".local"):
        candidate = candidate[:-6]
    return candidate


def _normalize_name_list(
    value: object,
    *,
    lower: bool = False,
) -> list[str] | None:
    if value is None:
        return None

    raw_values: list[object]
    if isinstance(value, str):
        raw_values = [value]
    elif isinstance(value, (list, tuple, set)):
        raw_values = list(value)
    else:
        raw_values = [value]

    normalized: list[str] = []
    seen: set[str] = set()
    for raw in raw_values:
        parts = (
            [segment for segment in raw.split(",")]
            if isinstance(raw, str)
            else [str(raw)]
        )
        for part in parts:
            candidate = part.strip().rstrip(".")
            if lower:
                candidate = candidate.lower()
            if not candidate or candidate in seen:
                continue
            seen.add(candidate)
            normalized.append(candidate)
    return normalized


class RuntimeNetworkPolicyOverride(BaseModel):
    network_role: NetworkRole | None = None
    allowed_ap_hosts: list[str] | None = None
    ap_selection: ApSelection | None = None

    @field_validator("allowed_ap_hosts", mode="before")
    @classmethod
    def _normalize_allowed_ap_hosts(
        cls, value: object
    ) -> list[str] | None:
        return _normalize_name_list(value, lower=True)

    def is_empty(self) -> bool:
        return (
            self.network_role is None
            and self.allowed_ap_hosts is None
            and self.ap_selection is None
        )


class FleetVehiclePreview(BaseModel):
    name: str
    kind: str | None = None
    mission: str | None = None
    mission_path: str | None = None
    auto_launch: bool | None = None
    debug: bool | None = None
    vision_debug: bool | None = None
    save_vision_milliseconds: int | None = None
    servo_only: bool | None = None
    camera_mount_offsets: list[float] = Field(default_factory=list)
    camera_input_transport: str | None = None
    camera_rotate_degrees: float | None = None
    camera_preprocess_hook: str | None = None
    px4_airframe_id: int | None = None
    px4_namespace: str | None = None
    payload_controller: str | None = None
    network_role: NetworkRole | None = None
    allowed_ap_vehicles: list[str] = Field(default_factory=list)
    ap_selection: ApSelection | None = None


class BuildSourcePayload(BaseModel):
    kind: Literal["none", "github", "local_artifact", "local_codebase"]
    summary: str
    github_source: Literal["release", "actions"] | None = None
    tag: str | None = None
    artifact_id: str | None = None
    run_id: str | None = None
    sha: str | None = None
    commit_subject: str | None = None
    name: str | None = None
    date: str | None = None
    download_url: str | None = None
    size_mb: float | None = None
    branch: str | None = None
    artifact_name: str | None = None
    workflow_name: str | None = None
    workflow_event: str | None = None
    workflow_conclusion: str | None = None
    local_artifact_path: str | None = None
    local_artifact_exists: bool | None = None
    local_artifact_size_bytes: int | None = None
    codebase_root: str | None = None
    fleet_file: str | None = None
    available_fleets: list[str] = Field(default_factory=list)
    fleet_options: list[str] = Field(default_factory=list)
    fleet_catalog_source: str | None = None
    fleet_catalog_error: str | None = None
    fleet_catalog_stale: bool = False
    fleet_exists: bool | None = None
    fleet_error: str | None = None
    fleet_vehicles: list[FleetVehiclePreview] = Field(default_factory=list)
    updated_at: str | None = None


class BuildSourceResponse(BaseModel):
    success: bool
    source: BuildSourcePayload | None = None
    output: str | None = None
    error: str | None = None


class FleetCatalogResponse(BaseModel):
    success: bool
    source_kind: str
    source_label: str | None = None
    selected_fleet_file: str | None = None
    available_fleets: list[str] = Field(default_factory=list)
    fleet_options: list[str] = Field(default_factory=list)
    fleet_catalog_source: str | None = None
    fleet_catalog_error: str | None = None
    fleet_catalog_stale: bool = False
    fleet_exists: bool | None = None
    fleet_error: str | None = None
    fleet_vehicles: list[FleetVehiclePreview] = Field(default_factory=list)
    error: str | None = None


class FleetConnectionSummary(BaseModel):
    success: bool
    connected: bool
    target: str | None = None
    info: str = ""
    error: str | None = None


class FleetCurrentBuildSummary(BaseModel):
    success: bool
    installed: bool
    info: str
    release_id: str | None = None


class FleetRuntimeSummary(BaseModel):
    success: bool
    running: bool
    state: str
    pid: str | None = None
    error: str | None = None


class FleetReadinessSummary(BaseModel):
    connected: bool = False
    build_installed: bool = False
    runtime_ready: bool = False
    vehicle_assigned: bool = False
    ready: bool = False
    notes: list[str] = Field(default_factory=list)


class FleetBoardDeviceResponse(LiveHardwareDeviceResponse):
    target_id: str | None = None
    vehicle_name: str | None = None
    fleet_vehicle: FleetVehiclePreview | None = None
    connection: FleetConnectionSummary | None = None
    current_build: FleetCurrentBuildSummary | None = None
    runtime: FleetRuntimeSummary | None = None
    readiness: FleetReadinessSummary | None = None


class FleetBoardSummary(BaseModel):
    total_devices: int = 0
    saved_devices: int = 0
    stale_devices: int = 0
    connected_devices: int = 0
    build_installed_devices: int = 0
    running_devices: int = 0
    ready_devices: int = 0


class FleetBoardResponse(BaseModel):
    success: bool
    build_source: BuildSourceResponse
    fleet_catalog: FleetCatalogResponse
    fleet_vehicles: list[FleetVehiclePreview] = Field(default_factory=list)
    devices: list[FleetBoardDeviceResponse] = Field(default_factory=list)
    summary: FleetBoardSummary = Field(default_factory=FleetBoardSummary)
    error: str | None = None


class FleetDeviceSelection(BaseModel):
    target_id: str | None = None
    hostname: str | None = None
    vehicle_name: str | None = None
    label: str | None = None
    pi_user: str | None = None
    deploy_root: str | None = None
    service_unit: str | None = None
    ssh_key: str | None = None
    ssh_pass: str | None = None
    network_role: NetworkRole | None = None
    allowed_ap_hosts: list[str] | None = None
    allowed_ap_vehicles: list[str] | None = None
    ap_selection: ApSelection | None = None

    @field_validator("allowed_ap_hosts", mode="before")
    @classmethod
    def _normalize_selection_allowed_ap_hosts(
        cls, value: object
    ) -> list[str] | None:
        return _normalize_name_list(value, lower=True)

    @field_validator("allowed_ap_vehicles", mode="before")
    @classmethod
    def _normalize_selection_allowed_ap_vehicles(
        cls, value: object
    ) -> list[str] | None:
        return _normalize_name_list(value)

    def network_policy_override(self) -> RuntimeNetworkPolicyOverride | None:
        override = RuntimeNetworkPolicyOverride(
            network_role=self.network_role,
            allowed_ap_hosts=self.allowed_ap_hosts,
            ap_selection=self.ap_selection,
        )
        return None if override.is_empty() else override

    def allowed_ap_vehicles_override(self) -> list[str] | None:
        return list(self.allowed_ap_vehicles or []) or None


class FleetBatchRequest(BaseModel):
    devices: list[FleetDeviceSelection] = Field(default_factory=list)
    session_assignments: dict[str, str] = Field(default_factory=dict)

    @field_validator("session_assignments", mode="before")
    @classmethod
    def _normalize_session_assignments(
        cls, value: object
    ) -> dict[str, str]:
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise TypeError("session_assignments must be a mapping.")

        normalized: dict[str, str] = {}
        for raw_host, raw_vehicle in value.items():
            host = _normalize_host_alias(raw_host)
            vehicle = str(raw_vehicle or "").strip()
            if not host or not vehicle:
                continue
            normalized[host] = vehicle
        return normalized


class FleetActionDeviceResponse(FleetBoardDeviceResponse):
    action: str
    success: bool
    attempted_release_id: str | None = None
    active_release_id: str | None = None
    rolled_back: bool = False
    output: str | None = None
    error: str | None = None


class FleetActionSummary(BaseModel):
    requested_devices: int = 0
    successful_devices: int = 0
    failed_devices: int = 0


class FleetActionResponse(BaseModel):
    success: bool
    action: str
    build_source: BuildSourceResponse
    fleet_catalog: FleetCatalogResponse
    fleet_vehicles: list[FleetVehiclePreview] = Field(default_factory=list)
    results: list[FleetActionDeviceResponse] = Field(default_factory=list)
    summary: FleetActionSummary = Field(default_factory=FleetActionSummary)
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
    mode: str
    module: str
    mission_target: str
    description: str | None = None
    required_vision_nodes: list[str] = Field(default_factory=list)
    transition_labels: list[str] = Field(default_factory=list)
    params: list[SchemaFieldResponse] = Field(default_factory=list)
    params_schema: dict[str, Any] = Field(default_factory=dict)


class MissionModeResponse(BaseModel):
    name: str
    mode: str
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
