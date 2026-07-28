import type { WifiScanResult, WifiStatus } from "./types.js";

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
      const response = await this.fetchImpl(`${this.baseUrl}/api/discovery`);
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
      { success: false, connections: [], error: "orchestrator unreachable" },
    );
  }

  async wifiScan(hostname: string): Promise<WifiScanResult> {
    return this.getJson<WifiScanResult>(
      `/api/wifi/scan?hostname=${encodeURIComponent(hostname)}`,
      { success: false, networks: [], error: "orchestrator unreachable" },
    );
  }

  async wifiConnect(hostname: string, ssid: string, password: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/wifi/connect", { hostname, ssid, password });
  }

  async wifiHotspot(hostname: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/wifi/hotspot", { hostname });
  }

  private async getJson<T>(path: string, onUnreachable: T): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`);
      if (!response.ok) return onUnreachable;
      return (await response.json()) as T;
    } catch {
      return onUnreachable;
    }
  }

  private async postJson<T extends { success: boolean; error?: string }>(
    path: string,
    body: unknown,
  ): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      return (await response.json()) as T;
    } catch {
      return { success: false, error: "orchestrator unreachable" } as T;
    }
  }
}
