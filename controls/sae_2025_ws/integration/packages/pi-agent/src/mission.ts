import type { LaunchStatus } from "@pennair/integration-core";
import { runCommand, type CommandRunner, type SimpleResult } from "./exec.js";
import { SERVICE_NAME } from "./service-name.js";

export async function missionStatus(run: CommandRunner = runCommand): Promise<LaunchStatus> {
  const active = await run("sudo", ["-n", "systemctl", "is-active", SERVICE_NAME], 8_000);
  const state = active.stdout.trim();

  if (state === "active") {
    const pidResult = await run(
      "sudo",
      ["-n", "systemctl", "show", "-p", "MainPID", "--value", SERVICE_NAME],
      8_000,
    );
    return { success: true, running: true, state: "running", pid: pidResult.stdout.trim() || "0" };
  }

  if (state) {
    // Any other real, non-empty state (inactive/failed/activating/deactivating/
    // reloading/etc.) means the unit exists and just isn't running -- matches
    // the old dashboard's bash `case` statement, which mapped every non-"active"
    // state to STOPPED via its `*)` catch-all, not just an enumerated list.
    return { success: true, running: false, state: "stopped" };
  }

  // Empty stdout means systemctl didn't actually report a state at all (e.g.
  // "sudo: a password is required" went to stderr with nothing on stdout) --
  // that's a real failure to query status, not a legitimate stopped state.
  return {
    success: false,
    running: false,
    state: "error",
    error: active.stderr || "systemctl produced no output",
  };
}

export async function prepareMission(run: CommandRunner = runCommand): Promise<SimpleResult> {
  const result = await run("sudo", ["-n", "systemctl", "restart", SERVICE_NAME], 25_000);
  if (result.code !== 0) {
    return { success: false, error: result.stderr || "Failed to start mission runtime" };
  }
  return { success: true, output: "Mission runtime started" };
}

export async function stopMission(run: CommandRunner = runCommand): Promise<SimpleResult> {
  const result = await run("sudo", ["-n", "systemctl", "stop", SERVICE_NAME], 20_000);
  if (result.code !== 0) {
    return { success: false, error: result.stderr || "Failed to stop mission runtime" };
  }
  return { success: true, output: "Mission runtime stopped" };
}
