import type {
  FileResult,
  FleetBoardResult,
  LaunchStatus,
  MissionNamesResult,
  SchemaExportResult,
  WifiScanResult,
  WifiStatus,
} from "./types.js";

export interface OrchestratorClientOptions {
  /** e.g. "http://localhost:8080" -- the one thing web/mobile clients talk to. */
  baseUrl: string;
  fetchImpl?: typeof fetch;
}

export interface DiscoveredPiSummary {
  hostname: string;
}

interface SimpleResult {
  success: boolean;
  output?: string;
  error?: string;
}

// Sized to exceed the orchestrator's own largest per-operation relay timeout
// to pi-agent (wifi connect, 80s -- see pi-agent-client.ts's
// WIFI_CONNECT_TIMEOUT_MS) plus margin, so the orchestrator's own timeout
// fires first and returns a graceful {success:false,...} JSON body rather
// than this client severing the connection first. Without an explicit
// timeout here, a wedged orchestrator (accepts the TCP connection but never
// responds) would hang every OrchestratorClient method forever -- fetch has
// no default overall-request timeout once connected, only a connect-timeout.
const DEFAULT_TIMEOUT_MS = 90_000;

const ORCHESTRATOR_UNREACHABLE_ERROR = "orchestrator unreachable";

/**
 * The only client web/mobile need -- talks to the orchestrator, never to a
 * Pi's pi-agent directly. The orchestrator does its own discovery and relays
 * device-ops calls to whichever Pi is named.
 */
export class OrchestratorClient {
  private readonly baseUrl: string;
  private readonly fetchImpl: typeof fetch;

  constructor(options: OrchestratorClientOptions) {
    this.baseUrl = options.baseUrl.replace(/\/$/, "");
    this.fetchImpl = options.fetchImpl ?? fetch;
  }

  async discoverPis(): Promise<DiscoveredPiSummary[]> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}/api/discovery`, {
        signal: AbortSignal.timeout(DEFAULT_TIMEOUT_MS),
      });
      if (!response.ok) return [];
      const body = (await response.json()) as { pis: DiscoveredPiSummary[] };
      return body.pis;
    } catch {
      return [];
    }
  }

  async wifiStatus(hostname: string): Promise<WifiStatus> {
    return this.getJson<WifiStatus>(
      `/api/wifi/status?hostname=${encodeURIComponent(hostname)}`,
      { success: false, connections: [], error: ORCHESTRATOR_UNREACHABLE_ERROR },
    );
  }

  async wifiScan(hostname: string): Promise<WifiScanResult> {
    return this.getJson<WifiScanResult>(
      `/api/wifi/scan?hostname=${encodeURIComponent(hostname)}`,
      { success: false, networks: [], error: ORCHESTRATOR_UNREACHABLE_ERROR },
    );
  }

  async wifiConnect(hostname: string, ssid: string, password: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/wifi/connect", { hostname, ssid, password });
  }

  async wifiHotspot(hostname: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/wifi/hotspot", { hostname });
  }

  async missionStatus(hostname: string): Promise<LaunchStatus> {
    return this.getJson<LaunchStatus>(
      `/api/mission/status?hostname=${encodeURIComponent(hostname)}`,
      { success: false, running: false, state: "offline", error: ORCHESTRATOR_UNREACHABLE_ERROR },
    );
  }

  async prepareMission(hostname: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/mission/prepare", { hostname });
  }

  async stopMission(hostname: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/mission/stop", { hostname });
  }

  async startMission(vehicleName: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/mission/start", { vehicleName });
  }

  async triggerFailsafe(vehicleName: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/mission/failsafe", { vehicleName });
  }

  /** WebSocket URL for streaming mission logs for the given Pi -- connect with a plain WebSocket. */
  missionLogsUrl(hostname: string): string {
    const wsBase = this.baseUrl.replace(/^http/, "ws");
    return `${wsBase}/ws/mission/logs?hostname=${encodeURIComponent(hostname)}`;
  }

  async schema(hostname: string): Promise<SchemaExportResult> {
    return this.getJson<SchemaExportResult>(
      `/api/schema?hostname=${encodeURIComponent(hostname)}`,
      { success: false, error: ORCHESTRATOR_UNREACHABLE_ERROR },
    );
  }

  async missionNames(hostname: string): Promise<MissionNamesResult> {
    return this.getJson<MissionNamesResult>(
      `/api/mission/mission-names?hostname=${encodeURIComponent(hostname)}`,
      { success: false, missions: [], error: ORCHESTRATOR_UNREACHABLE_ERROR },
    );
  }

  async readMissionFile(hostname: string, name: string): Promise<FileResult> {
    return this.getJson<FileResult>(
      `/api/mission/mission-file?hostname=${encodeURIComponent(hostname)}&name=${encodeURIComponent(name)}`,
      { success: false, error: ORCHESTRATOR_UNREACHABLE_ERROR },
    );
  }

  async writeMissionFile(hostname: string, name: string, content: string): Promise<FileResult> {
    return this.postJson<FileResult>("/api/mission/mission-file", { hostname, name, content });
  }

  async readFleetFile(hostname: string, name: string): Promise<FileResult> {
    return this.getJson<FileResult>(
      `/api/fleet/fleet-file?hostname=${encodeURIComponent(hostname)}&name=${encodeURIComponent(name)}`,
      { success: false, error: ORCHESTRATOR_UNREACHABLE_ERROR },
    );
  }

  async writeFleetFile(hostname: string, name: string, content: string): Promise<FileResult> {
    return this.postJson<FileResult>("/api/fleet/fleet-file", { hostname, name, content });
  }

  async fleetBoard(): Promise<FleetBoardResult> {
    return this.getJson<FleetBoardResult>("/api/fleet/board", {
      success: false,
      devices: [],
      summary: { totalDevices: 0, connectedDevices: 0, buildInstalledDevices: 0, runningDevices: 0, readyDevices: 0 },
      error: ORCHESTRATOR_UNREACHABLE_ERROR,
    });
  }

  private async getJson<T>(path: string, onUnreachable: T, timeoutMs: number = DEFAULT_TIMEOUT_MS): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`, {
        signal: AbortSignal.timeout(timeoutMs),
      });
      if (!response.ok) return onUnreachable;
      return (await response.json()) as T;
    } catch {
      return onUnreachable;
    }
  }

  private async postJson<T extends { success: boolean; error?: string }>(
    path: string,
    body: unknown,
    timeoutMs: number = DEFAULT_TIMEOUT_MS,
  ): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
        signal: AbortSignal.timeout(timeoutMs),
      });
      return (await response.json()) as T;
    } catch {
      return { success: false, error: ORCHESTRATOR_UNREACHABLE_ERROR } as T;
    }
  }
}
