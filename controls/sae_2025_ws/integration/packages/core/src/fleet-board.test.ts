import { describe, expect, it } from "vitest";
import { computeFleetReadiness, summarizeFleetBoard } from "./fleet-board.js";
import type { FleetBoardDevice } from "./types.js";

describe("computeFleetReadiness", () => {
  it("is ready when connected, build installed, and runtime is stopped", () => {
    const result = computeFleetReadiness({ connected: true, buildInstalled: true, runtimeState: "stopped" });
    expect(result).toEqual({ ready: true, notes: [] });
  });

  it("is ready when runtime is not_prepared too", () => {
    const result = computeFleetReadiness({ connected: true, buildInstalled: true, runtimeState: "not_prepared" });
    expect(result.ready).toBe(true);
  });

  it("is not ready and notes an unreachable Pi", () => {
    const result = computeFleetReadiness({ connected: false, buildInstalled: false, runtimeState: "offline" });
    expect(result.ready).toBe(false);
    expect(result.notes).toContain("Pi unreachable");
    // An unreachable Pi's runtime state can't meaningfully be "not ready to
    // launch" -- that note is reserved for a connected-but-not-idle Pi.
    expect(result.notes).not.toContain("Runtime not ready to launch");
  });

  it("notes a missing build separately from connectivity", () => {
    const result = computeFleetReadiness({ connected: true, buildInstalled: false, runtimeState: "stopped" });
    expect(result.ready).toBe(false);
    expect(result.notes).toEqual(["No installed build"]);
  });

  it("notes a running/error runtime as not ready to launch, only when connected", () => {
    const running = computeFleetReadiness({ connected: true, buildInstalled: true, runtimeState: "running" });
    expect(running.ready).toBe(false);
    expect(running.notes).toEqual(["Runtime not ready to launch"]);

    const errored = computeFleetReadiness({ connected: true, buildInstalled: true, runtimeState: "error" });
    expect(errored.ready).toBe(false);
    expect(errored.notes).toEqual(["Runtime not ready to launch"]);
  });
});

describe("summarizeFleetBoard", () => {
  function device(overrides: Partial<FleetBoardDevice>): FleetBoardDevice {
    return {
      hostname: "air-01.local",
      connected: false,
      buildInstalled: false,
      runtimeState: "offline",
      runtimeRunning: false,
      ready: false,
      notes: [],
      ...overrides,
    };
  }

  it("rolls up counts across devices", () => {
    const devices = [
      device({ hostname: "a", connected: true, buildInstalled: true, runtimeRunning: true, ready: false }),
      device({ hostname: "b", connected: true, buildInstalled: true, ready: true }),
      device({ hostname: "c", connected: false }),
    ];
    expect(summarizeFleetBoard(devices)).toEqual({
      totalDevices: 3,
      connectedDevices: 2,
      buildInstalledDevices: 2,
      runningDevices: 1,
      readyDevices: 1,
    });
  });

  it("handles an empty fleet", () => {
    expect(summarizeFleetBoard([])).toEqual({
      totalDevices: 0,
      connectedDevices: 0,
      buildInstalledDevices: 0,
      runningDevices: 0,
      readyDevices: 0,
    });
  });
});
