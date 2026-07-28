import type { LaunchStatus } from "@pennair/integration-core";
import { runCommand, type CommandRunner } from "./exec.js";

const SERVICE_NAME = process.env.MISSION_SERVICE_NAME ?? "pennair-autonomy.service";

interface SimpleResult {
  success: boolean;
  output?: string;
  error?: string;
}

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

  if (["inactive", "failed", "activating", "deactivating"].includes(state)) {
    return { success: true, running: false, state: "stopped" };
  }

  return {
    success: false,
    running: false,
    state: "error",
    error: active.stderr || `Unexpected systemctl state: ${state || "(empty)"}`,
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
