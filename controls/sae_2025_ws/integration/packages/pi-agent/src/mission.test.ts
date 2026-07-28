import { describe, expect, it } from "vitest";
import type { CommandRunner, ExecResult } from "./exec.js";
import { missionStatus, prepareMission, stopMission } from "./mission.js";

function ok(stdout: string): ExecResult {
  return { code: 0, stdout, stderr: "" };
}

function fail(stderr: string): ExecResult {
  return { code: 1, stdout: "", stderr };
}

function fakeRunner(responses: Record<string, ExecResult>): CommandRunner {
  return async (command, args) => {
    const key = [command, ...args].join(" ");
    return responses[key] ?? fail(`no fake response registered for: ${key}`);
  };
}

describe("missionStatus", () => {
  it("reports running with a pid when the service is active", async () => {
    const run = fakeRunner({
      "sudo -n systemctl is-active pennair-autonomy.service": ok("active\n"),
      "sudo -n systemctl show -p MainPID --value pennair-autonomy.service": ok("12345\n"),
    });

    const status = await missionStatus(run);
    expect(status).toEqual({ success: true, running: true, state: "running", pid: "12345" });
  });

  it("reports stopped for any non-active, non-empty state (matches the old bash case statement's catch-all)", async () => {
    // Not just the enumerated inactive/failed/activating/deactivating --
    // systemd can report other states too (e.g. "reloading"), and the old
    // dashboard's `case ... *) echo STOPPED ;;` mapped all of them to stopped.
    for (const state of ["inactive", "failed", "activating", "deactivating", "reloading", "some-future-state"]) {
      const run = fakeRunner({
        "sudo -n systemctl is-active pennair-autonomy.service": ok(`${state}\n`),
      });
      const status = await missionStatus(run);
      expect(status).toEqual({ success: true, running: false, state: "stopped" });
    }
  });

  it("reports an error state when systemctl produces no output at all (e.g. a sudo password prompt)", async () => {
    const run = fakeRunner({
      "sudo -n systemctl is-active pennair-autonomy.service": fail("sudo: a password is required"),
    });
    const status = await missionStatus(run);
    expect(status.success).toBe(false);
    expect(status.state).toBe("error");
    expect(status.error).toContain("password");
  });
});

describe("prepareMission", () => {
  it("restarts the service and reports success", async () => {
    const run = fakeRunner({ "sudo -n systemctl restart pennair-autonomy.service": ok("") });
    expect(await prepareMission(run)).toEqual({ success: true, output: "Mission runtime started" });
  });

  it("reports failure with the systemctl error", async () => {
    const run = fakeRunner({
      "sudo -n systemctl restart pennair-autonomy.service": fail("Unit not found."),
    });
    expect(await prepareMission(run)).toEqual({ success: false, error: "Unit not found." });
  });
});

describe("stopMission", () => {
  it("stops the service and reports success", async () => {
    const run = fakeRunner({ "sudo -n systemctl stop pennair-autonomy.service": ok("") });
    expect(await stopMission(run)).toEqual({ success: true, output: "Mission runtime stopped" });
  });
});
