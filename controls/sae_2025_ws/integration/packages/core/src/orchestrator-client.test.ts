import { describe, expect, it, vi } from "vitest";
import { OrchestratorClient } from "./orchestrator-client.js";

function jsonResponse(body: unknown, ok = true): Response {
  return { ok, json: async () => body } as Response;
}

describe("OrchestratorClient", () => {
  it("discoverPis returns the list of Pis the orchestrator reports", async () => {
    const fetchImpl = vi.fn(async () => jsonResponse({ pis: [{ hostname: "air-01.local" }] }));
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.discoverPis()).toEqual([{ hostname: "air-01.local" }]);
  });

  it("discoverPis returns an empty list if the orchestrator is unreachable", async () => {
    const fetchImpl = vi.fn(async () => {
      throw new Error("network error");
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.discoverPis()).toEqual([]);
  });

  it("wifiStatus passes the hostname through as a query param", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      expect(url.toString()).toBe("http://localhost:8080/api/wifi/status?hostname=air-01.local");
      return jsonResponse({ success: true, connections: [] });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    const status = await client.wifiStatus("air-01.local");
    expect(status.success).toBe(true);
  });

  it("wifiConnect posts hostname/ssid/password as JSON", async () => {
    const fetchImpl = vi.fn(async (_url: string | URL, init?: RequestInit) => {
      expect(init?.method).toBe("POST");
      expect(JSON.parse(init?.body as string)).toEqual({
        hostname: "air-01.local",
        ssid: "pennair_5G",
        password: "hunter2",
      });
      return jsonResponse({ success: true, output: "Pi connected to pennair_5G" });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    const result = await client.wifiConnect("air-01.local", "pennair_5G", "hunter2");
    expect(result).toEqual({ success: true, output: "Pi connected to pennair_5G" });
  });

  it("strips a trailing slash from baseUrl", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      expect(url.toString()).toBe("http://localhost:8080/api/discovery");
      return jsonResponse({ pis: [] });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080/", fetchImpl: fetchImpl as unknown as typeof fetch });

    await client.discoverPis();
  });
});
