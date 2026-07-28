import type { WifiScanResult, WifiStatus } from "@pennair/integration-core";

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
      const response = await this.fetchImpl(`${this.baseUrl}/health`);
      return response.ok;
    } catch {
      return false;
    }
  }

  async wifiStatus(): Promise<WifiStatus> {
    return this.getJson<WifiStatus>("/api/wifi/status", {
      success: false,
      connections: [],
      error: "unreachable",
    });
  }

  async wifiScan(): Promise<WifiScanResult> {
    return this.getJson<WifiScanResult>("/api/wifi/scan", {
      success: false,
      networks: [],
      error: "unreachable",
    });
  }

  async wifiConnect(ssid: string, password: string): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/wifi/connect", { ssid, password });
  }

  async wifiHotspot(): Promise<SimpleResult> {
    return this.postJson<SimpleResult>("/api/wifi/hotspot", {});
  }

  private async getJson<T>(path: string, onUnreachable: T): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`);
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
  ): Promise<T> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
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
