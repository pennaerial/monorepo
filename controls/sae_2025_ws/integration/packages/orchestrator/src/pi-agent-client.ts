import { randomBytes } from "node:crypto";
import { Readable } from "node:stream";
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
const DEPLOY_CURRENT_TIMEOUT_MS = 10_000;
const DEPLOY_ROLLBACK_TIMEOUT_MS = 30_000; // matches pi-agent's own activate/health-check-rollback window (8s poll) plus margin
// Large and variable by design -- artifact size isn't bounded, so this can't
// be sized off a fixed pi-agent-side exec duration the way the other
// timeouts above are. 10 minutes comfortably covers a multi-hundred-MB
// transfer even on a slow travel-router link.
const DEPLOY_UPLOAD_TIMEOUT_MS = 600_000;

export interface DeployUploadFields {
  sourceType?: string;
  sourceLabel?: string;
  vehicleName?: string;
  fleetFile?: string;
}

export interface DeployUploadResult {
  success: boolean;
  error?: string;
  releaseId?: string;
}

export interface CurrentBuildResult {
  installed: boolean;
  releaseId?: string;
  extractedAt?: string;
  sourceType?: string;
  sourceLabel?: string;
  error?: string;
}

/** Builds a multipart/form-data body as an async byte stream, so the incoming client upload can be relayed to pi-agent without ever buffering the whole artifact in memory. */
async function* multipartBody(
  fields: DeployUploadFields,
  filename: string,
  fileStream: AsyncIterable<Buffer>,
  boundary: string,
): AsyncGenerator<Buffer> {
  const encoder = new TextEncoder();
  for (const [key, value] of Object.entries(fields)) {
    if (value === undefined) continue;
    yield Buffer.from(encoder.encode(`--${boundary}\r\nContent-Disposition: form-data; name="${key}"\r\n\r\n${value}\r\n`));
  }
  yield Buffer.from(
    encoder.encode(
      `--${boundary}\r\nContent-Disposition: form-data; name="file"; filename="${filename}"\r\nContent-Type: application/octet-stream\r\n\r\n`,
    ),
  );
  for await (const chunk of fileStream) {
    yield chunk;
  }
  yield Buffer.from(encoder.encode(`\r\n--${boundary}--\r\n`));
}

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

  async currentBuild(): Promise<CurrentBuildResult> {
    return this.getJson<CurrentBuildResult>(
      "/api/deploy/current",
      { installed: false, error: "unreachable" },
      DEPLOY_CURRENT_TIMEOUT_MS,
    );
  }

  async rollbackDeploy(): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/deploy/rollback", {}, DEPLOY_ROLLBACK_TIMEOUT_MS);
  }

  async uploadDeploy(
    filename: string,
    fileStream: AsyncIterable<Buffer>,
    fields: DeployUploadFields,
  ): Promise<DeployUploadResult> {
    const boundary = `----pennair-relay-${randomBytes(8).toString("hex")}`;
    try {
      const response = await this.fetchImpl(`${this.baseUrl}/api/deploy/upload`, {
        method: "POST",
        headers: { "Content-Type": `multipart/form-data; boundary=${boundary}` },
        body: Readable.from(multipartBody(fields, filename, fileStream, boundary)) as unknown as ReadableStream,
        // Node's fetch (undici) requires this when the body is a stream.
        duplex: "half",
        signal: AbortSignal.timeout(DEPLOY_UPLOAD_TIMEOUT_MS),
      } as RequestInit);
      return (await response.json()) as DeployUploadResult;
    } catch (error) {
      return { success: false, error: (error as Error).message };
    }
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
