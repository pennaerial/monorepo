export interface PiTarget {
  targetId: string;
  label: string;
  hostname: string;
  vehicleName: string;
  enabled: boolean;
}

export type NetworkRole = "default" | "ap" | "client";
export type ApSelection = "ordered_then_strongest" | "strongest";

export interface WifiConnection {
  name: string;
  type: string;
  device: string;
  state: string;
}

export interface WifiStatus {
  success: boolean;
  isHotspot?: boolean;
  currentWifi?: string;
  currentMode?: string;
  effectiveRole?: string;
  connections: WifiConnection[];
  error?: string;
}

export interface WifiNetwork {
  ssid: string;
  signal: number;
  security: string;
}

export interface WifiScanResult {
  success: boolean;
  networks: WifiNetwork[];
  error?: string;
}

export type MissionPhase =
  | "idle"
  | "preparing"
  | "running"
  | "stopping"
  | "error"
  | "offline";

export type LaunchState = "running" | "stopped" | "not_prepared" | "error" | "offline";

export interface MissionRuntimeState {
  phase: MissionPhase;
  launchState: LaunchState;
  running: boolean;
  pid?: string;
  message?: string;
  error?: string;
  updatedAt: number;
}

export interface BuildSource {
  kind: "none" | "github" | "local_artifact" | "local_codebase";
  summary: string;
  fleetFile?: string;
  updatedAt?: string;
}
