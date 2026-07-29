import { describe, expect, it } from "vitest";
import { PiAgentClient } from "./pi-agent-client.js";
import { fetchFleetBoard } from "./fleet-board.js";

/**
 * `fetchFleetBoard`'s per-Pi fan-out (fleet-board.ts) uses `Promise.all`
 * (not `allSettled`) across `isHealthy()`/`missionStatus()`/`currentBuild()`,
 * on the stated assumption that `PiAgentClient`'s own methods already
 * degrade to a graceful `{success:false, ...}` shape instead of throwing.
 * If that assumption were wrong, one malformed/weird Pi response would
 * reject the outer `Promise.all` and wipe the *entire* board -- not just
 * that one device's row.
 *
 * These tests exercise the specific edge the reasoning depends on:
 * `getJson`'s `await response.json()` call throwing (malformed JSON body)
 * needs to be caught by `getJson`'s own try/catch rather than propagating.
 * Reading pi-agent-client.ts confirms `await response.json()` sits *inside*
 * the try block for every `getJson`-backed method, and `isHealthy()` has its
 * own independent try/catch -- so the assumption holds. These tests prove
 * it holds rather than just re-reading the source.
 */

function malformedJsonFetch(): typeof fetch {
  return (async () =>
    new Response("not valid json{{{", {
      status: 200,
      headers: { "content-type": "application/json" },
    })) as unknown as typeof fetch;
}

describe("PiAgentClient resolves gracefully (never rejects) on a malformed JSON body", () => {
  it("missionStatus() falls back to the unreachable shape instead of rejecting", async () => {
    const client = new PiAgentClient({ hostname: "x", port: 1, fetchImpl: malformedJsonFetch() });
    await expect(client.missionStatus()).resolves.toEqual({
      success: false,
      running: false,
      state: "offline",
      error: "unreachable",
    });
  });

  it("currentBuild() falls back to the unreachable shape instead of rejecting", async () => {
    const client = new PiAgentClient({ hostname: "x", port: 1, fetchImpl: malformedJsonFetch() });
    await expect(client.currentBuild()).resolves.toEqual({
      success: false,
      installed: false,
      error: "unreachable",
    });
  });

  it("isHealthy() resolves false instead of rejecting when fetchImpl itself throws synchronously", async () => {
    const throwingFetch = (() => {
      throw new Error("DNS resolution failed");
    }) as unknown as typeof fetch;
    const client = new PiAgentClient({ hostname: "x", port: 1, fetchImpl: throwingFetch });
    await expect(client.isHealthy()).resolves.toBe(false);
  });
});

describe("fetchFleetBoard survives a malformed JSON body from one discovered Pi", () => {
  it("does not reject the whole board when a discovered Pi's /api/mission/status returns malformed JSON", async () => {
    const fetchImpl = (async (url: Parameters<typeof fetch>[0]) => {
      const u = String(url);
      if (u === "http://air-01.local:8090/health") {
        return new Response(JSON.stringify({ status: "ok" }), { status: 200 });
      }
      if (u.startsWith("http://air-01.local:8090/api/mission/status")) {
        // Malformed body: response.ok is true, but response.json() will throw.
        return new Response("{not json", { status: 200 });
      }
      if (u.startsWith("http://air-01.local:8090/api/deploy/current")) {
        return new Response(JSON.stringify({ success: true, installed: true, info: "Build 1" }), { status: 200 });
      }
      throw new Error(`unexpected fetch ${u}`);
    }) as typeof fetch;

    // Proves the Promise.all-based fan-out doesn't reject: if it did, this
    // whole `await` would throw and the test would fail with an unhandled
    // rejection rather than reaching the assertions below.
    const result = await fetchFleetBoard({
      prefixes: ["air"],
      suffixes: [1],
      port: 8090,
      discoveryTimeoutMs: 1000,
      fetchImpl,
      piAgentClientFor: (hostname) => new PiAgentClient({ hostname, port: 8090, fetchImpl }),
    });

    expect(result.success).toBe(true);
    expect(result.devices).toHaveLength(1);
    expect(result.devices[0].connected).toBe(true);
    // missionStatus() degraded to the unreachable fallback for this one probe.
    expect(result.devices[0].runtimeState).toBe("offline");
  });
});
