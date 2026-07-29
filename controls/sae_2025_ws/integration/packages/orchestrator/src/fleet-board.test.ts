import { describe, expect, it } from "vitest";
import { PiAgentClient } from "./pi-agent-client.js";
import { fetchFleetBoard } from "./fleet-board.js";

function jsonResponse(body: unknown): Response {
  return new Response(JSON.stringify(body), { status: 200, headers: { "content-type": "application/json" } });
}

describe("fetchFleetBoard", () => {
  it("discovers Pis and fans out health/mission/build probes into a board with a summary", async () => {
    const fetchImpl = (async (url: Parameters<typeof fetch>[0]) => {
      const u = String(url);
      if (u === "http://air-01.local:8090/health") return jsonResponse({ status: "ok" });
      if (u === "http://air-02.local:8090/health") throw new Error("unreachable");
      if (u.startsWith("http://air-01.local:8090/api/mission/status")) {
        return jsonResponse({ success: true, running: false, state: "stopped" });
      }
      if (u.startsWith("http://air-01.local:8090/api/deploy/current")) {
        return jsonResponse({ success: true, installed: true, info: "Build 42", releaseId: "rel-1" });
      }
      throw new Error(`unexpected fetch ${u}`);
    }) as typeof fetch;

    const result = await fetchFleetBoard({
      prefixes: ["air"],
      suffixes: [1, 2],
      port: 8090,
      discoveryTimeoutMs: 1000,
      fetchImpl,
      piAgentClientFor: (hostname) => new PiAgentClient({ hostname, port: 8090, fetchImpl }),
    });

    expect(result.success).toBe(true);
    // Only air-01 is discovered -- air-02's /health probe throws.
    expect(result.devices).toHaveLength(1);
    expect(result.devices[0]).toMatchObject({
      hostname: "air-01.local",
      connected: true,
      buildInstalled: true,
      releaseId: "rel-1",
      buildInfo: "Build 42",
      runtimeState: "stopped",
      runtimeRunning: false,
      ready: true,
      notes: [],
    });
    expect(result.summary).toEqual({
      totalDevices: 1,
      connectedDevices: 1,
      buildInstalledDevices: 1,
      runningDevices: 0,
      readyDevices: 1,
    });
  });

  it("returns an empty board when nothing is discovered", async () => {
    const fetchImpl = (async () => {
      throw new Error("unreachable");
    }) as typeof fetch;

    const result = await fetchFleetBoard({ prefixes: ["air"], suffixes: [1], port: 8090, discoveryTimeoutMs: 1000, fetchImpl });

    expect(result).toEqual({
      success: true,
      devices: [],
      summary: { totalDevices: 0, connectedDevices: 0, buildInstalledDevices: 0, runningDevices: 0, readyDevices: 0 },
    });
  });

  it("includes a discovered-but-since-unreachable Pi as a not-ready row rather than failing the whole board", async () => {
    const discoveryFetch = (async (url: Parameters<typeof fetch>[0]) => {
      if (String(url) === "http://air-01.local:8090/health") return jsonResponse({ status: "ok" });
      throw new Error("unreachable");
    }) as typeof fetch;
    const piAgentFetch = (async () => {
      throw new Error("connection refused");
    }) as typeof fetch;

    const result = await fetchFleetBoard({
      prefixes: ["air"],
      suffixes: [1],
      port: 8090,
      discoveryTimeoutMs: 1000,
      fetchImpl: discoveryFetch,
      piAgentClientFor: (hostname) => new PiAgentClient({ hostname, port: 8090, fetchImpl: piAgentFetch }),
    });

    expect(result.success).toBe(true);
    expect(result.devices).toHaveLength(1);
    expect(result.devices[0].connected).toBe(false);
    expect(result.devices[0].ready).toBe(false);
    expect(result.devices[0].notes).toContain("Pi unreachable");
  });

  it("marks a connected Pi with no installed build as not ready, with a clear note", async () => {
    const fetchImpl = (async (url: Parameters<typeof fetch>[0]) => {
      const u = String(url);
      if (u === "http://air-01.local:8090/health") return jsonResponse({ status: "ok" });
      if (u.startsWith("http://air-01.local:8090/api/mission/status")) {
        return jsonResponse({ success: true, running: false, state: "not_prepared" });
      }
      if (u.startsWith("http://air-01.local:8090/api/deploy/current")) {
        return jsonResponse({ success: true, installed: false });
      }
      throw new Error(`unexpected fetch ${u}`);
    }) as typeof fetch;

    const result = await fetchFleetBoard({
      prefixes: ["air"],
      suffixes: [1],
      port: 8090,
      discoveryTimeoutMs: 1000,
      fetchImpl,
      piAgentClientFor: (hostname) => new PiAgentClient({ hostname, port: 8090, fetchImpl }),
    });

    expect(result.devices[0].connected).toBe(true);
    expect(result.devices[0].buildInstalled).toBe(false);
    expect(result.devices[0].ready).toBe(false);
    expect(result.devices[0].notes).toEqual(["No installed build"]);
  });

  it("dedupes a hostname that discovery returns more than once, rather than double-counting it", async () => {
    // A duplicated prefix (e.g. misconfigured DISCOVERY_PREFIXES) makes
    // discoverPis's own candidate cross-product probe and return the same
    // hostname twice -- fetchFleetBoard must not fan out (or count) it twice.
    let probeCount = 0;
    const fetchImpl = (async (url: Parameters<typeof fetch>[0]) => {
      const u = String(url);
      if (u === "http://air-01.local:8090/health") {
        probeCount += 1;
        return jsonResponse({ status: "ok" });
      }
      if (u.startsWith("http://air-01.local:8090/api/mission/status")) {
        return jsonResponse({ success: true, running: false, state: "stopped" });
      }
      if (u.startsWith("http://air-01.local:8090/api/deploy/current")) {
        return jsonResponse({ success: true, installed: true });
      }
      throw new Error(`unexpected fetch ${u}`);
    }) as typeof fetch;

    const result = await fetchFleetBoard({
      prefixes: ["air", "air"], // duplicated on purpose
      suffixes: [1],
      port: 8090,
      discoveryTimeoutMs: 1000,
      fetchImpl,
      piAgentClientFor: (hostname) => new PiAgentClient({ hostname, port: 8090, fetchImpl }),
    });

    expect(result.devices).toHaveLength(1);
    expect(result.summary.totalDevices).toBe(1);
    // 2 discovery /health probes (one per duplicated candidate -- discovery
    // itself isn't deduped) + 1 more from fetchFleetBoard's own isHealthy()
    // fan-out for the single deduped device -- not 2 fan-out calls, which
    // would mean the board fanned out to the same Pi twice.
    expect(probeCount).toBe(3);
  });

  it("forwards fetchImpl into the default per-Pi client when piAgentClientFor isn't supplied", async () => {
    // Regression test: the default `clientFor` used to construct a bare
    // `new PiAgentClient({ hostname, port })` with no fetchImpl, so a caller
    // supplying only fetchImpl (discovery gets faked) would silently fall
    // through to the real global fetch for the per-Pi probes.
    const fetchImpl = (async (url: Parameters<typeof fetch>[0]) => {
      const u = String(url);
      if (u === "http://air-01.local:8090/health") return jsonResponse({ status: "ok" });
      if (u.startsWith("http://air-01.local:8090/api/mission/status")) {
        return jsonResponse({ success: true, running: false, state: "stopped" });
      }
      if (u.startsWith("http://air-01.local:8090/api/deploy/current")) {
        return jsonResponse({ success: true, installed: true, info: "Build 42", releaseId: "rel-1" });
      }
      throw new Error(`unexpected fetch ${u}`);
    }) as typeof fetch;

    // No piAgentClientFor supplied -- exercises the default clientFor path.
    const result = await fetchFleetBoard({ prefixes: ["air"], suffixes: [1], port: 8090, discoveryTimeoutMs: 1000, fetchImpl });

    expect(result.devices).toHaveLength(1);
    expect(result.devices[0]).toMatchObject({ connected: true, buildInstalled: true, releaseId: "rel-1" });
  });
});
