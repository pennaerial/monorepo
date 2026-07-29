import { describe, expect, it } from "vitest";
import { computeFleetReadiness, summarizeFleetBoard } from "./fleet-board.js";
import type { FleetBoardDevice } from "./types.js";

describe("computeFleetReadiness: connected=true but runtimeState=offline", () => {
  // This combination looks contradictory at a glance, but it's real and
  // reachable: pi-agent-client.ts's missionStatus() falls back to exactly
  // `state: "offline"` when ITS OWN probe fails/times out (see its
  // `onUnreachable` default), independently of isHealthy()'s /health probe.
  // fetchFleetBoard fans both calls out via Promise.all with *different*
  // per-call timeouts (HEALTH_TIMEOUT_MS=5s vs MISSION_STATUS_TIMEOUT_MS=
  // 20s), so it's entirely possible for /health to succeed (connected: true)
  // while /api/mission/status independently times out or 5xxs on the same
  // poll (runtimeState: "offline"). Not a synthetic edge case -- it's the
  // real fallback value substituted by missionStatus() itself.
  it("is not ready, and reuses the same note as a genuinely running/errored runtime", () => {
    const result = computeFleetReadiness({ connected: true, buildInstalled: true, runtimeState: "offline" });
    expect(result.ready).toBe(false);
    // Documents current behavior rather than asserting a "should": the note
    // produced here is identical to the one produced when runtimeState is
    // "running" or "error" (see fleet-board.test.ts's existing case). There
    // is no distinct note for "connected, but we couldn't determine runtime
    // state" -- an operator sees "Runtime not ready to launch" for both a Pi
    // that's mid-mission and one whose status probe merely timed out this
    // poll, which is a real ambiguity in the board UI, if not a crash bug.
    expect(result.notes).toEqual(["Runtime not ready to launch"]);
  });
});

describe("summarizeFleetBoard: duplicate hostnames in the device list", () => {
  function device(overrides: Partial<FleetBoardDevice>): FleetBoardDevice {
    return {
      hostname: "air-01.local",
      connected: true,
      buildInstalled: true,
      runtimeState: "stopped",
      runtimeRunning: false,
      ready: true,
      notes: [],
      ...overrides,
    };
  }

  // discoverPis() (packages/core/src/discovery.ts) builds its candidate
  // list as the cross product of `prefixes` x `suffixes`
  // (candidateHostnames), with no de-duplication of `prefixes` itself. The
  // orchestrator's DISCOVERY_PREFIXES env var is split on "," with no
  // trim/filter (packages/orchestrator/src/index.ts's discoveryParams()),
  // so a duplicated prefix (e.g. "air,air" from a copy-paste config
  // mistake) makes discoverPis() probe and return the exact same hostname
  // twice. fetchFleetBoard has no hostname-uniqueness safeguard downstream
  // of discovery, and summarizeFleetBoard just filters/counts the array it
  // is given -- so a single physical Pi would be double-counted in every
  // summary bucket it belongs to, and FleetBoard.tsx would render two
  // <li key={device.hostname}> cards sharing a React key.
  it("double-counts a hostname that appears twice, rather than de-duplicating by hostname", () => {
    const devices = [device({}), device({})];
    expect(summarizeFleetBoard(devices)).toEqual({
      totalDevices: 2,
      connectedDevices: 2,
      buildInstalledDevices: 2,
      runningDevices: 0,
      readyDevices: 2,
    });
  });
});
