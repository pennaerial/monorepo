import type { FleetBoardDevice, FleetBoardSummary, LaunchState } from "./types.js";

export interface FleetReadinessInput {
  connected: boolean;
  buildInstalled: boolean;
  runtimeState: LaunchState;
}

/**
 * Mirrors the old dashboard's `FleetReadinessSummary` computation
 * (`connected && build_installed && runtime in {stopped, not_prepared}`),
 * minus the old system's `vehicle_assigned` criterion -- that required a
 * persisted Pi-to-fleet-vehicle assignment the old board itself only ever
 * populated from ad hoc client-side session state (never persisted
 * server-side), and cross-referencing fleet YAML vehicle entries isn't
 * built yet in this rewrite. Scoped out of Phase 5, not silently dropped.
 */
export function computeFleetReadiness(input: FleetReadinessInput): { ready: boolean; notes: string[] } {
  const notes: string[] = [];
  if (!input.connected) {
    notes.push("Pi unreachable");
  }
  if (!input.buildInstalled) {
    notes.push("No installed build");
  }
  const runtimeReady = input.runtimeState === "stopped" || input.runtimeState === "not_prepared";
  if (input.connected && !runtimeReady) {
    notes.push("Runtime not ready to launch");
  }

  return {
    ready: input.connected && input.buildInstalled && runtimeReady,
    notes,
  };
}

export function summarizeFleetBoard(devices: FleetBoardDevice[]): FleetBoardSummary {
  return {
    totalDevices: devices.length,
    connectedDevices: devices.filter((device) => device.connected).length,
    buildInstalledDevices: devices.filter((device) => device.buildInstalled).length,
    runningDevices: devices.filter((device) => device.runtimeRunning).length,
    readyDevices: devices.filter((device) => device.ready).length,
  };
}
