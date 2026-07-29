import { existsSync } from "node:fs";
import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import * as tar from "tar";
import { afterAll, afterEach, beforeAll, beforeEach, describe, expect, it, vi } from "vitest";
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

  it("trims whitespace and de-dupes a repeated/malformed DISCOVERY_PREFIXES value, avoiding a double-counted Pi", async () => {
    const fetchMock = vi.fn(async (url: string | URL) => {
      const reachable = url.toString() === "http://air-01.local:8090/health";
      return { ok: reachable } as Response;
    });
    vi.stubGlobal("fetch", fetchMock);
    process.env.DISCOVERY_PREFIXES = "air, air,  ,air";
    process.env.DISCOVERY_SUFFIX_MAX = "1";
    process.env.PI_AGENT_PORT = "8090";

    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/discovery" });

    expect(response.json()).toEqual({ pis: [{ hostname: "air-01.local" }] });

    delete process.env.DISCOVERY_PREFIXES;
    delete process.env.DISCOVERY_SUFFIX_MAX;
    delete process.env.PI_AGENT_PORT;
  });
});

describe("orchestrator fleet board endpoint", () => {
  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("discovers a Pi and fans out into a board response with a readiness summary", async () => {
    const fetchMock = vi.fn(async (url: string | URL) => {
      const u = url.toString();
      if (u === "http://air-01.local:8090/health") {
        return new Response(JSON.stringify({ status: "ok" }), { status: 200 });
      }
      if (u.startsWith("http://air-01.local:8090/api/mission/status")) {
        return new Response(JSON.stringify({ success: true, running: false, state: "stopped" }), { status: 200 });
      }
      if (u.startsWith("http://air-01.local:8090/api/deploy/current")) {
        return new Response(JSON.stringify({ success: true, installed: true, info: "Build 42", releaseId: "rel-1" }), {
          status: 200,
        });
      }
      throw new Error(`unexpected fetch ${u}`);
    });
    vi.stubGlobal("fetch", fetchMock);
    process.env.DISCOVERY_PREFIXES = "air";
    process.env.DISCOVERY_SUFFIX_MAX = "1";
    process.env.PI_AGENT_PORT = "8090";

    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/fleet/board" });

    expect(response.statusCode).toBe(200);
    const body = response.json();
    expect(body.devices).toEqual([
      {
        hostname: "air-01.local",
        connected: true,
        buildInstalled: true,
        releaseId: "rel-1",
        buildInfo: "Build 42",
        runtimeState: "stopped",
        runtimeRunning: false,
        ready: true,
        notes: [],
      },
    ]);
    expect(body.summary.readyDevices).toBe(1);

    delete process.env.DISCOVERY_PREFIXES;
    delete process.env.DISCOVERY_SUFFIX_MAX;
    delete process.env.PI_AGENT_PORT;
  });
});

describe("orchestrator build-source selection endpoints", () => {
  let stateDir: string;
  const originalStateDir = process.env.ORCHESTRATOR_STATE_DIR;

  beforeEach(async () => {
    stateDir = await mkdtemp(join(tmpdir(), "orchestrator-state-test-"));
    process.env.ORCHESTRATOR_STATE_DIR = stateDir;
  });

  afterEach(async () => {
    process.env.ORCHESTRATOR_STATE_DIR = originalStateDir;
    await rm(stateDir, { recursive: true, force: true });
  });

  it("defaults to kind:none", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/deploy/source" });
    expect(response.json()).toMatchObject({ kind: "none" });
  });

  it("selects a github release source, persists it, and resolves its fleet catalog error when there's no download url", async () => {
    const orchestrator = buildOrchestratorServer();
    const select = await orchestrator.inject({
      method: "POST",
      url: "/api/deploy/source/github",
      payload: { githubSource: "release", tag: "build-42", name: "Build 42" },
    });
    expect(select.json()).toMatchObject({ kind: "github", tag: "build-42" });

    const reread = await orchestrator.inject({ method: "GET", url: "/api/deploy/source" });
    expect(reread.json()).toMatchObject({ kind: "github", tag: "build-42" });

    const catalog = await orchestrator.inject({ method: "GET", url: "/api/deploy/fleet-catalog" });
    expect(catalog.json()).toMatchObject({ catalog: {}, error: expect.stringContaining("download URL") });
  });

  it("selects a local codebase source and resolves fleet yaml files directly from disk", async () => {
    const codebaseRoot = join(stateDir, "repo");
    await mkdir(join(codebaseRoot, "src", "uav", "uav", "fleets"), { recursive: true });
    await writeFile(join(codebaseRoot, "src", "uav", "uav", "fleets", "example_fleet.yaml"), "vehicles: []", "utf-8");

    const orchestrator = buildOrchestratorServer();
    await orchestrator.inject({
      method: "POST",
      url: "/api/deploy/source/local-codebase",
      payload: { codebaseRoot },
    });

    const catalog = await orchestrator.inject({ method: "GET", url: "/api/deploy/fleet-catalog" });
    expect(catalog.json()).toMatchObject({ catalog: { "example_fleet.yaml": expect.any(String) }, error: null });
  });

  it("clear() resets to kind:none", async () => {
    const orchestrator = buildOrchestratorServer();
    await orchestrator.inject({
      method: "POST",
      url: "/api/deploy/source/github",
      payload: { githubSource: "release", tag: "build-42" },
    });
    const cleared = await orchestrator.inject({ method: "POST", url: "/api/deploy/source/clear" });
    expect(cleared.json()).toMatchObject({ kind: "none" });
  });

  it("reports the GITHUB_REPO-not-configured error from /api/deploy/builds", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/deploy/builds" });
    expect(response.json()).toMatchObject({ success: false, error: expect.stringContaining("not configured") });
  });

  it("accepts a real multipart local-artifact upload and selects it as the build source", async () => {
    const stagingDir = join(stateDir, "staging");
    await mkdir(join(stagingDir, "uav", "fleets"), { recursive: true });
    await writeFile(join(stagingDir, "uav", "fleets", "example_fleet.yaml"), "vehicles: []", "utf-8");
    const artifactPath = join(stateDir, "local-build.tar.gz");
    await tar.c({ file: artifactPath, cwd: stagingDir, gzip: true }, ["uav"]);

    const orchestrator = buildOrchestratorServer();
    await orchestrator.listen({ port: 0, host: "127.0.0.1" });
    const address = orchestrator.server.address();
    if (typeof address === "string" || address === null) {
      throw new Error("expected orchestrator to listen on a TCP port");
    }

    const form = new FormData();
    const fileBuffer = await import("node:fs/promises").then((fs) => fs.readFile(artifactPath));
    form.set("file", new Blob([fileBuffer]), "local-build.tar.gz");
    const response = await fetch(`http://127.0.0.1:${address.port}/api/deploy/source/local-artifact`, {
      method: "POST",
      body: form,
    });
    const body = (await response.json()) as { kind: string; artifactName: string };
    expect(body).toMatchObject({ kind: "local_artifact", artifactName: "local-build.tar.gz" });

    const catalog = await orchestrator.inject({ method: "GET", url: "/api/deploy/fleet-catalog" });
    expect(catalog.json()).toMatchObject({ catalog: { "example_fleet.yaml": expect.any(String) } });

    await orchestrator.close();
  });
});

describe("orchestrator deploy relay to a real local pi-agent instance", () => {
  let scratchRoot: string;
  let piAgent: ReturnType<typeof buildPiAgentServer>;
  let piAgentPort: number;
  const originalPiAgentPortEnv = process.env.PI_AGENT_PORT;
  const originalDeployRoot = process.env.DEPLOY_ROOT;

  beforeAll(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "orchestrator-deploy-relay-test-"));
    process.env.DEPLOY_ROOT = scratchRoot;
    piAgent = buildPiAgentServer();
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
    process.env.DEPLOY_ROOT = originalDeployRoot;
    await rm(scratchRoot, { recursive: true, force: true });
  });

  afterEach(async () => {
    await rm(`${scratchRoot}/releases`, { recursive: true, force: true });
    await rm(`${scratchRoot}/current`, { force: true });
    await rm(`${scratchRoot}/previous`, { force: true });
  });

  it("relays /api/deploy/current to the real pi-agent", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/deploy/current?hostname=127.0.0.1" });
    expect(response.json()).toMatchObject({ installed: false });
  });

  it("relays a real multipart upload end-to-end: client -> orchestrator -> real pi-agent, which genuinely extracts it", async () => {
    const stagingDir = join(scratchRoot, "staging");
    await mkdir(join(stagingDir, "install"), { recursive: true });
    await writeFile(join(stagingDir, "install", "marker.txt"), "v1", "utf-8");
    const artifactPath = join(scratchRoot, "upload-test.tar.gz");
    await tar.c({ file: artifactPath, cwd: stagingDir, gzip: true }, ["install"]);

    const orchestrator = buildOrchestratorServer();
    await orchestrator.listen({ port: 0, host: "127.0.0.1" });
    const address = orchestrator.server.address();
    if (typeof address === "string" || address === null) {
      throw new Error("expected orchestrator to listen on a TCP port");
    }

    const form = new FormData();
    form.set("hostname", "127.0.0.1");
    form.set("sourceType", "github");
    form.set("sourceLabel", "build-42");
    form.set("vehicleName", "air-01");
    const fileBuffer = await import("node:fs/promises").then((fs) => fs.readFile(artifactPath));
    form.set("file", new Blob([fileBuffer]), "upload-test.tar.gz");

    const response = await fetch(`http://127.0.0.1:${address.port}/api/deploy/upload`, {
      method: "POST",
      body: form,
    });
    const body = (await response.json()) as { success: boolean; releaseId?: string };

    // Real graceful failure expected: no real systemd/sudo on this dev machine.
    expect(body.success).toBe(false);
    expect(body.releaseId).toBeTruthy();
    expect(existsSync(`${scratchRoot}/current/install/marker.txt`)).toBe(true);

    await orchestrator.close();
  });

  it("returns 400 when no hostname is provided to the upload endpoint", async () => {
    const orchestrator = buildOrchestratorServer();
    await orchestrator.listen({ port: 0, host: "127.0.0.1" });
    const address = orchestrator.server.address();
    if (typeof address === "string" || address === null) {
      throw new Error("expected orchestrator to listen on a TCP port");
    }
    const form = new FormData();
    form.set("file", new Blob([Buffer.from("test")]), "artifact.tar.gz");
    const response = await fetch(`http://127.0.0.1:${address.port}/api/deploy/upload`, {
      method: "POST",
      body: form,
    });
    expect(response.status).toBe(400);
    await orchestrator.close();
  });
});

describe("orchestrator mission/fleet-file/schema relay to a real local pi-agent instance", () => {
  let scratchRoot: string;
  let piAgent: ReturnType<typeof buildPiAgentServer>;
  let piAgentPort: number;
  const originalPiAgentPortEnv = process.env.PI_AGENT_PORT;
  const originalDeployRoot = process.env.DEPLOY_ROOT;

  beforeAll(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "orchestrator-mission-fleet-relay-test-"));
    process.env.DEPLOY_ROOT = scratchRoot;

    const releaseDir = join(scratchRoot, "releases", "20260101-000000-test");
    await mkdir(join(releaseDir, "install", "uav", "share", "uav", "missions"), { recursive: true });
    await mkdir(join(releaseDir, "install", "uav", "share", "uav", "fleets"), { recursive: true });
    await writeFile(
      join(releaseDir, "install", "uav", "share", "uav", "missions", "patrol.yaml"),
      "modes:\n  start:\n    mode: uav.LandingMode\n",
      "utf-8",
    );
    await writeFile(
      join(releaseDir, "install", "uav", "share", "uav", "fleets", "example_fleet.yaml"),
      "backend:\n  kind: hardware\nvehicles: []\n",
      "utf-8",
    );
    const { symlink } = await import("node:fs/promises");
    await symlink(releaseDir, join(scratchRoot, "current"));

    piAgent = buildPiAgentServer();
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
    process.env.DEPLOY_ROOT = originalDeployRoot;
    await rm(scratchRoot, { recursive: true, force: true });
  });

  it("relays /api/mission/mission-names to the real pi-agent", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/mission/mission-names?hostname=127.0.0.1" });
    expect(response.json()).toEqual({ success: true, missions: ["patrol"] });
  });

  it("returns 400 when hostname is missing from mission-names", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/mission/mission-names" });
    expect(response.statusCode).toBe(400);
  });

  it("relays mission file read/write round trips through to the real pi-agent", async () => {
    const orchestrator = buildOrchestratorServer();
    const read = await orchestrator.inject({
      method: "GET",
      url: "/api/mission/mission-file?hostname=127.0.0.1&name=patrol",
    });
    expect(read.json().content).toContain("uav.LandingMode");

    const write = await orchestrator.inject({
      method: "POST",
      url: "/api/mission/mission-file",
      payload: { hostname: "127.0.0.1", name: "patrol", content: "modes:\n  start:\n    mode: uav.NavGPSMode\n" },
    });
    expect(write.json()).toEqual({ success: true });

    const reread = await orchestrator.inject({
      method: "GET",
      url: "/api/mission/mission-file?hostname=127.0.0.1&name=patrol",
    });
    expect(reread.json().content).toContain("uav.NavGPSMode");
  });

  it("relays fleet file read/write round trips through to the real pi-agent", async () => {
    const orchestrator = buildOrchestratorServer();
    const read = await orchestrator.inject({
      method: "GET",
      url: "/api/fleet/fleet-file?hostname=127.0.0.1&name=example_fleet",
    });
    expect(read.json().content).toContain("hardware");

    const write = await orchestrator.inject({
      method: "POST",
      url: "/api/fleet/fleet-file",
      payload: { hostname: "127.0.0.1", name: "example_fleet", content: "backend:\n  kind: hardware\nvehicles:\n  - name: air-01\n" },
    });
    expect(write.json()).toEqual({ success: true });

    const reread = await orchestrator.inject({
      method: "GET",
      url: "/api/fleet/fleet-file?hostname=127.0.0.1&name=example_fleet",
    });
    expect(reread.json().content).toContain("air-01");
  });

  it("relays /api/schema, which genuinely fails without a real ROS2 environment on this dev machine", async () => {
    const orchestrator = buildOrchestratorServer();
    const response = await orchestrator.inject({ method: "GET", url: "/api/schema?hostname=127.0.0.1" });
    const body = response.json();
    expect(body.success).toBe(false);
    expect(body.error).toBeTruthy();
  });
});
