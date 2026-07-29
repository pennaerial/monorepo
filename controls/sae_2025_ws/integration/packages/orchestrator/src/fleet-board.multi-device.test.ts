import { mkdir, mkdtemp, rm, symlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterAll, afterEach, beforeAll, describe, expect, it, vi } from "vitest";
import { buildServer as buildPiAgentServer } from "@pennair/pi-agent";
import { buildServer as buildOrchestratorServer } from "./index.js";

/**
 * Every existing fleet-board test (core and orchestrator) exercises exactly
 * one discovered device. This exercises the real GET /api/fleet/board
 * endpoint against *two* real local pi-agent instances on different ports,
 * to check that the board genuinely aggregates multiple devices (not just
 * fans a single-element array through, and doesn't conflate/overwrite rows)
 * rather than just relying on synthetic single-device unit tests.
 *
 * Note: both real pi-agent instances run in this same Node test process and
 * therefore share one `process.env` -- pi-agent's own deploy.ts reads
 * DEPLOY_ROOT fresh per-request via `deployPaths()`, so two instances can't
 * safely be given two *different* DEPLOY_ROOT values here without a request
 * race (whichever value is current when each instance's request handler
 * runs). Both instances are given the same DEPLOY_ROOT/installed release
 * instead -- what this test is actually checking is aggregation-by-hostname
 * correctness (two distinct rows, correct per-device connectivity, correct
 * rolled-up summary counts), not per-device build-state divergence.
 */
describe("orchestrator fleet board endpoint against two real local pi-agent instances", () => {
  let scratchRoot: string;
  let piAgentA: ReturnType<typeof buildPiAgentServer>;
  let piAgentB: ReturnType<typeof buildPiAgentServer>;
  let portA: number;
  let portB: number;
  const originalDeployRoot = process.env.DEPLOY_ROOT;

  beforeAll(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "fleet-board-multi-"));

    const releaseDir = join(scratchRoot, "releases", "20260101-000000-test");
    await mkdir(releaseDir, { recursive: true });
    await writeFile(join(releaseDir, "RELEASE_METADATA.json"), JSON.stringify({ releaseId: "rel-shared" }), "utf-8");
    await symlink(releaseDir, join(scratchRoot, "current"));
    process.env.DEPLOY_ROOT = scratchRoot;

    piAgentA = buildPiAgentServer();
    await piAgentA.listen({ port: 0, host: "127.0.0.1" });
    const addressA = piAgentA.server.address();
    if (typeof addressA === "string" || addressA === null) {
      throw new Error("expected pi-agent A to listen on a TCP port");
    }
    portA = addressA.port;

    piAgentB = buildPiAgentServer();
    await piAgentB.listen({ port: 0, host: "127.0.0.1" });
    const addressB = piAgentB.server.address();
    if (typeof addressB === "string" || addressB === null) {
      throw new Error("expected pi-agent B to listen on a TCP port");
    }
    portB = addressB.port;
  });

  afterAll(async () => {
    await piAgentA.close();
    await piAgentB.close();
    process.env.DEPLOY_ROOT = originalDeployRoot;
    await rm(scratchRoot, { recursive: true, force: true });
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("aggregates both devices into one board with a correct rolled-up summary", async () => {
    const hostnameToPort: Record<string, number> = {
      "air-01.local": portA,
      "air-02.local": portB,
    };
    // Capture the real fetch before stubbing the global -- routedFetch must
    // call through to it, not recurse into the stub it's about to become.
    const realFetch = fetch;
    const routedFetch = (async (url: Parameters<typeof fetch>[0], init?: RequestInit) => {
      const original = new URL(String(url));
      const port = hostnameToPort[original.hostname];
      if (port === undefined) throw new Error(`no route for ${original.hostname}`);
      const target = new URL(`${original.pathname}${original.search}`, `http://127.0.0.1:${port}`);
      return realFetch(target, init);
    }) as typeof fetch;

    process.env.DISCOVERY_PREFIXES = "air";
    process.env.DISCOVERY_SUFFIX_MAX = "2";
    process.env.PI_AGENT_PORT = "8090"; // irrelevant to routing; routedFetch ignores the literal port

    vi.stubGlobal("fetch", routedFetch);
    try {
      const orchestrator = buildOrchestratorServer();
      const response = await orchestrator.inject({ method: "GET", url: "/api/fleet/board" });

      expect(response.statusCode).toBe(200);
      const body = response.json();
      expect(body.success).toBe(true);
      expect(body.devices).toHaveLength(2);

      const hostnames = (body.devices as Array<{ hostname: string }>).map((d) => d.hostname).sort();
      expect(hostnames).toEqual(["air-01.local", "air-02.local"]);

      for (const device of body.devices as Array<{
        connected: boolean;
        buildInstalled: boolean;
        releaseId?: string;
      }>) {
        expect(device.connected).toBe(true);
        expect(device.buildInstalled).toBe(true);
        expect(device.releaseId).toBe("20260101-000000-test");
      }

      // Real graceful failure expected: no real systemd/sudo on this dev
      // machine, so missionStatus() reports state:"error" for both real
      // pi-agent instances -- neither device is "ready", but both got there
      // via a genuinely independent round trip, not a value copied across
      // rows.
      expect(body.summary).toEqual({
        totalDevices: 2,
        connectedDevices: 2,
        buildInstalledDevices: 2,
        runningDevices: 0,
        readyDevices: 0,
      });
    } finally {
      delete process.env.DISCOVERY_PREFIXES;
      delete process.env.DISCOVERY_SUFFIX_MAX;
      delete process.env.PI_AGENT_PORT;
    }
  });
});
