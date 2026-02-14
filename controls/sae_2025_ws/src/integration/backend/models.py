from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field


class ErrorResponse(BaseModel):
    success: Literal[False] = False
    error: str


class MessageResponse(BaseModel):
    success: Literal[True] = True
    output: str


class ConfigPayload(BaseModel):
    pi_user: str
    pi_host: str
    remote_dir: str
    ssh_key: str
    ssh_pass: str
    github_repo: str
    hotspot_name: str


class ConfigResponse(BaseModel):
    success: Literal[True] = True
    config: ConfigPayload


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


class BuildListItem(BaseModel):
    tag: str
    sha: str
    name: str
    date: str
    download_url: str | None = None
    size_mb: float | None = None


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
