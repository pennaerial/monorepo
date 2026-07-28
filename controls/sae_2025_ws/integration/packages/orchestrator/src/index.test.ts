import { afterAll, afterEach, beforeAll, describe, expect, it, vi } from "vitest";
import { buildServer as buildPiAgentServer } from "@pennair/pi-agent";
import { buildServer as buildOrchestratorServer } from "./index.js";

describe("orchestrator wifi relay, against a real local pi-agent instance", () => {
  const piAgent = buildPiAgentServer();
  let piAgentPort: number;
  const originalPiAgentPortEnv = process.env.PI_AGENT_PORT;

  beforeAll(async () => {
    await piAgent.listen({ port: 0, host: "127.0.0.1" });
    const address = piAgent.server.address();
    if (typeof address === "string" || address === null) {
      throw new Error("expected pi-agent to listen on a TCP port");
    }
    piAgentPort = address.port;
    process.env.PI_AGENT_PORT = String(piAgentPort);
  });

  afterAll(async () => {
    await piAgent.close();
    process.env.PI_AGENT_PORT = originalPiAgentPortEnv;
  });

  it("relays /api/wifi/status to the real pi-agent (nmcli absent in this environment, so a graceful failure is the correct real response)", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({
      method: "GET",
      url: "/api/wifi/status?hostname=127.0.0.1",
    });
    expect(response.statusCode).toBe(200);
    const body = response.json();
    // We're not asserting success:true here -- there's no real nmcli on this
    // dev machine. The point is that the orchestrator actually reached the
    // real pi-agent process over HTTP and relayed back whatever it said,
    // rather than the request failing to even reach it.
    expect(body).toHaveProperty("success");
    expect(body).toHaveProperty("connections");
  });

  it("returns 400 when hostname is missing", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/wifi/status" });
    expect(response.statusCode).toBe(400);
  });

  it("relays /api/wifi/connect POST bodies through to pi-agent", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({
      method: "POST",
      url: "/api/wifi/connect",
      payload: { hostname: "127.0.0.1", ssid: "some_network", password: "hunter2" },
    });
    expect(response.statusCode).toBe(200);
    const body = response.json();
    // Again: no real nmcli here, so success:false is expected and correct --
    // what matters is the request actually reached pi-agent's real handler.
    expect(body).toHaveProperty("success", false);
  });
});

describe("orchestrator discovery endpoint", () => {
  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("wires discoverPis with the configured prefixes/port and returns its result", async () => {
    const fetchMock = vi.fn(async (url: string | URL) => {
      const reachable = url.toString() === "http://air-01.local:8090/health";
      return { ok: reachable } as Response;
    });
    vi.stubGlobal("fetch", fetchMock);
    process.env.DISCOVERY_PREFIXES = "air";
    process.env.DISCOVERY_SUFFIX_MAX = "2";
    process.env.PI_AGENT_PORT = "8090";

    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/discovery" });

    expect(response.statusCode).toBe(200);
    expect(response.json()).toEqual({ pis: [{ hostname: "air-01.local" }] });

    delete process.env.DISCOVERY_PREFIXES;
    delete process.env.DISCOVERY_SUFFIX_MAX;
    delete process.env.PI_AGENT_PORT;
  });
});
