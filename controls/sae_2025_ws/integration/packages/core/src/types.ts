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
  policySource?: string;
  runtimeActive?: boolean;
  missionStarted?: boolean;
  travelRouterLocked?: boolean;
  switchingDisabled?: boolean;
  localApProfile?: string;
  travelRouterProfile?: string;
  allowedApHosts?: string[];
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

export interface LaunchStatus {
  success: boolean;
  running: boolean;
  state: LaunchState;
  pid?: string;
  error?: string;
}

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

// ---------------------------------------------------------------------------
// Mission/fleet YAML editor types (Phase 4)
// ---------------------------------------------------------------------------

export type SchemaFieldType =
  | "boolean"
  | "bool"
  | "integer"
  | "number"
  | "int"
  | "float"
  | "tuple"
  | "array"
  | "object"
  | "union"
  | "list"
  | "dict"
  | "string";

export interface SchemaField {
  name: string;
  schemaType?: SchemaFieldType;
  choices?: string[];
  default?: unknown;
  defaultKind?: "missing" | "present";
  description?: string;
  annotation?: string;
  required?: boolean;
  nullable?: boolean;
}

export type SchemaFieldInputKind = "select" | "boolean" | "number" | "tuple" | "block" | "text";

export interface ModeMetadata {
  name: string;
  mode: string;
  classPath?: string;
  target?: "uav" | "payload" | "";
  description?: string;
  requiredVisionNodes?: string[];
  transitionLabels?: string[];
  params: SchemaField[];
}

/** Flat lookup of mode registry entries, keyed by both the public mode id and the class path (mirrors `normalizeModeRegistry`). */
export type ModeRegistryByTarget = Record<string, ModeMetadata[]>;

export interface TransitionEntry {
  key: string;
  value: string;
}

export interface MissionMode {
  name: string;
  mode: string;
  paramsRaw: string;
  transitionsRaw: string;
  transitions: TransitionEntry[];
  target: "uav" | "payload" | "";
  hasParams: boolean;
  hasTransitions: boolean;
}

export interface ParsedMissionDocument {
  rawText: string;
  /** Raw text of any top-level content before the `modes:` key (header comments, other keys) -- preserved verbatim on render instead of being discarded. */
  preamble: string;
  modes: MissionMode[];
  selectedTarget: "uav" | "payload" | "";
  warnings: string[];
}

export interface FleetBackend {
  kind?: string;
  px4_path?: string;
  [key: string]: unknown;
}

export interface FleetVehicle {
  name?: string;
  kind?: string;
  mission?: string;
  px4_airframe_id?: unknown;
  px4_namespace?: unknown;
  [key: string]: unknown;
}

export interface FleetEditorDocument {
  backend: FleetBackend;
  defaults: Record<string, unknown>;
  vehicles: FleetVehicle[];
}

export interface ParsedFleetDocument {
  rawText: string;
  doc: FleetEditorDocument | null;
  error: string;
}

export type ObjectPathSegment = string | number;

export interface FleetSchemaSection {
  name: string;
  label?: string;
  fields: SchemaField[];
}

export interface FleetSchema {
  sections: FleetSchemaSection[];
  backendKinds?: string[];
}

export interface MissionCatalogEntry {
  name: string;
  target: "uav" | "payload" | "";
}

export interface SchemaExportResult {
  success: boolean;
  schema?: {
    modes: ModeRegistryByTarget;
    fleet: FleetSchema;
    missions: MissionCatalogEntry[];
    availableFleets: string[];
  };
  error?: string;
}

export interface MissionNamesResult {
  success: boolean;
  missions: string[];
  error?: string;
}

export interface FileResult {
  success: boolean;
  content?: string;
  error?: string;
}
