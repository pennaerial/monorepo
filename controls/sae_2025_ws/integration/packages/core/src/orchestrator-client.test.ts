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

  it("startMission posts vehicleName as JSON", async () => {
    const fetchImpl = vi.fn(async (url: string | URL, init?: RequestInit) => {
      expect(url.toString()).toBe("http://localhost:8080/api/mission/start");
      expect(JSON.parse(init?.body as string)).toEqual({ vehicleName: "air-01" });
      return jsonResponse({ success: true, output: "Mission runtime running" });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.startMission("air-01")).toEqual({ success: true, output: "Mission runtime running" });
  });

  it("triggerFailsafe posts vehicleName as JSON", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      expect(url.toString()).toBe("http://localhost:8080/api/mission/failsafe");
      return jsonResponse({ success: true, output: "failsafe triggered" });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.triggerFailsafe("air-01")).toEqual({ success: true, output: "failsafe triggered" });
  });

  it("missionLogsUrl converts http(s) to ws(s) and passes hostname as a query param", () => {
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080" });
    expect(client.missionLogsUrl("air-01.local")).toBe(
      "ws://localhost:8080/ws/mission/logs?hostname=air-01.local",
    );
  });

  it("missionLogsUrl handles an https baseUrl", () => {
    const client = new OrchestratorClient({ baseUrl: "https://ops.example.com" });
    expect(client.missionLogsUrl("air-01.local")).toBe(
      "wss://ops.example.com/ws/mission/logs?hostname=air-01.local",
    );
  });

  it("schema passes the hostname through as a query param", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      expect(url.toString()).toBe("http://localhost:8080/api/schema?hostname=air-01.local");
      return jsonResponse({ success: true, schema: { modes: {}, fleet: { sections: [] }, missions: [], availableFleets: [] } });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    const result = await client.schema("air-01.local");
    expect(result.success).toBe(true);
  });

  it("missionNames passes the hostname through as a query param", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      expect(url.toString()).toBe("http://localhost:8080/api/mission/mission-names?hostname=air-01.local");
      return jsonResponse({ success: true, missions: ["patrol"] });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.missionNames("air-01.local")).toEqual({ success: true, missions: ["patrol"] });
  });

  it("readMissionFile passes hostname and name through as query params", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      expect(url.toString()).toBe("http://localhost:8080/api/mission/mission-file?hostname=air-01.local&name=patrol");
      return jsonResponse({ success: true, content: "modes: {}" });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.readMissionFile("air-01.local", "patrol")).toEqual({ success: true, content: "modes: {}" });
  });

  it("writeMissionFile posts hostname/name/content as JSON", async () => {
    const fetchImpl = vi.fn(async (url: string | URL, init?: RequestInit) => {
      expect(url.toString()).toBe("http://localhost:8080/api/mission/mission-file");
      expect(JSON.parse(init?.body as string)).toEqual({ hostname: "air-01.local", name: "patrol", content: "modes: {}" });
      return jsonResponse({ success: true });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.writeMissionFile("air-01.local", "patrol", "modes: {}")).toEqual({ success: true });
  });

  it("readFleetFile passes hostname and name through as query params", async () => {
    const fetchImpl = vi.fn(async (url: string | URL) => {
      expect(url.toString()).toBe("http://localhost:8080/api/fleet/fleet-file?hostname=air-01.local&name=example_fleet");
      return jsonResponse({ success: true, content: "vehicles: []" });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.readFleetFile("air-01.local", "example_fleet")).toEqual({ success: true, content: "vehicles: []" });
  });

  it("writeFleetFile posts hostname/name/content as JSON", async () => {
    const fetchImpl = vi.fn(async (url: string | URL, init?: RequestInit) => {
      expect(url.toString()).toBe("http://localhost:8080/api/fleet/fleet-file");
      expect(JSON.parse(init?.body as string)).toEqual({
        hostname: "air-01.local",
        name: "example_fleet",
        content: "vehicles: []",
      });
      return jsonResponse({ success: true });
    });
    const client = new OrchestratorClient({ baseUrl: "http://localhost:8080", fetchImpl: fetchImpl as unknown as typeof fetch });

    expect(await client.writeFleetFile("air-01.local", "example_fleet", "vehicles: []")).toEqual({ success: true });
  });
});
