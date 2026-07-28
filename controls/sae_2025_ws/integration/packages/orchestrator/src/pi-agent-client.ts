import type { LaunchStatus, WifiScanResult, WifiStatus } from "@pennair/integration-core";

export interface PiAgentClientOptions {
  hostname: string;
  port: number;
  fetchImpl?: typeof fetch;
}

interface SimpleResult {
  success: boolean;
  output?: string;
  error?: string;
}

// Sized relative to pi-agent's own internal nmcli exec timeouts for the same
// operation (policyStatus 15s + the operation's own nmcli calls), plus a
// margin -- these must stay >= pi-agent's worst case, or the orchestrator
// would abort a request pi-agent is still legitimately working on. Without
// an explicit timeout here, a fetch has no default overall request timeout
// once connected (only a connect-timeout), so a wedged pi-agent could hang
// the orchestrator indefinitely.
const HEALTH_TIMEOUT_MS = 5_000;
const WIFI_STATUS_TIMEOUT_MS = 35_000; // policyStatus(15s) + con show(15s)
const WIFI_SCAN_TIMEOUT_MS = 25_000; // dev wifi list(20s)
const WIFI_CONNECT_TIMEOUT_MS = 80_000; // policyStatus(15s) + con down(15s) + connect(30s) + rollback con up(15s)
const WIFI_HOTSPOT_TIMEOUT_MS = 50_000; // policyStatus(15s) + disconnect(15s) + con up(15s)
const MISSION_STATUS_TIMEOUT_MS = 20_000; // is-active(8s) + show MainPID(8s)
const MISSION_PREPARE_TIMEOUT_MS = 30_000; // systemctl restart(25s)
const MISSION_STOP_TIMEOUT_MS = 25_000; // systemctl stop(20s)

/** Thin HTTP client the orchestrator uses to reach one Pi's pi-agent daemon. */
export class PiAgentClient {
  private readonly baseUrl: string;
  private readonly fetchImpl: typeof fetch;

  constructor(options: PiAgentClientOptions) {
    this.baseUrl = `http://${options.hostname}:${options.port}`;
    this.fetchImpl = options.fetchImpl ?? fetch;
  }

  async isHealthy(): Promise<boolean> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}/health`, {
        signal: AbortSignal.timeout(HEALTH_TIMEOUT_MS),
      });
      return response.ok;
    } catch {
      return false;
    }
  }

  async wifiStatus(): Promise<WifiStatus> {
    return this.getJson<WifiStatus>(
      "/api/wifi/status",
      { success: false, connections: [], error: "unreachable" },
      WIFI_STATUS_TIMEOUT_MS,
    );
  }

  async wifiScan(): Promise<WifiScanResult> {
    return this.getJson<WifiScanResult>(
      "/api/wifi/scan",
      { success: false, networks: [], error: "unreachable" },
      WIFI_SCAN_TIMEOUT_MS,
    );
  }

  async wifiConnect(ssid: string, password: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>(
      "/api/wifi/connect",
      { ssid, password },
      WIFI_CONNECT_TIMEOUT_MS,
    );
  }

  async wifiHotspot(): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/wifi/hotspot", {}, WIFI_HOTSPOT_TIMEOUT_MS);
  }

  async missionStatus(): Promise<LaunchStatus> {
    return this.getJson<LaunchStatus>(
      "/api/mission/status",
      { success: false, running: false, state: "offline", error: "unreachable" },
      MISSION_STATUS_TIMEOUT_MS,
    );
  }

  async prepareMission(): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/mission/prepare", {}, MISSION_PREPARE_TIMEOUT_MS);
  }

  async stopMission(): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/mission/stop", {}, MISSION_STOP_TIMEOUT_MS);
  }

  private async getJson<T>(path: string, onUnreachable: T, timeoutMs: number): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`, {
        signal: AbortSignal.timeout(timeoutMs),
      });
      if (!response.ok) {
        return onUnreachable;
      }
      return (await response.json()) as T;
    } catch {
      return onUnreachable;
    }
  }

  private async postJson<T extends { success: boolean; error?: string }>(
    path: string,
    body: unknown,
    timeoutMs: number,
  ): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
        signal: AbortSignal.timeout(timeoutMs),
      });
      const parsed = (await response.json()) as T;
      if (!response.ok && parsed.success === undefined) {
        return { success: false, error: "unreachable" } as T;
      }
      return parsed;
    } catch {
      return { success: false, error: "unreachable" } as T;
    }
  }
}
