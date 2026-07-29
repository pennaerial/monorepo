import { mkdir, mkdtemp, rm, symlink } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import type { CommandRunner, ExecResult } from "./exec.js";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { exportSchema } from "./schema-export.js";

function ok(stdout: string): ExecResult {
  return { code: 0, stdout, stderr: "" };
}
function fail(stderr: string): ExecResult {
  return { code: 1, stdout: "", stderr };
}

describe("exportSchema", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-schema-export-test-"));
    paths = deployPaths(scratchRoot);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  it("reports 'No build deployed' when there's no current release, without ever invoking the runner", async () => {
    let called = false;
    const run: CommandRunner = async () => {
      called = true;
      return ok("{}");
    };
    const result = await exportSchema(run, paths, "irrelevant.py");
    expect(result).toEqual({ success: false, error: "No build deployed" });
    expect(called).toBe(false);
  });

  it("sources ROS2 and the release's overlay, then runs the given script against --root, and parses stdout as JSON", async () => {
    const releaseDir = join(scratchRoot, "releases", "20260101-000000-test");
    await mkdir(releaseDir, { recursive: true });
    await symlink(releaseDir, paths.currentLink);

    let capturedCommand = "";
    const run: CommandRunner = async (command, args) => {
      expect(command).toBe("bash");
      expect(args[0]).toBe("-c");
      capturedCommand = args[1];
      return ok(JSON.stringify({ modes: {}, fleet: { sections: [] }, missions: [] }));
    };

    const result = await exportSchema(run, paths, "/path/to/schema_export.py");
    expect(result.success).toBe(true);
    expect(result.schema).toEqual({ modes: {}, fleet: { sections: [] }, missions: [] });
    expect(capturedCommand).toContain("source /opt/ros/jazzy/setup.bash");
    expect(capturedCommand).toContain(`source "${releaseDir}/install/setup.bash"`);
    expect(capturedCommand).toContain(`python3 "/path/to/schema_export.py" --root "${releaseDir}"`);
  });

  it("surfaces a graceful error when the script exits non-zero", async () => {
    const releaseDir = join(scratchRoot, "releases", "20260101-000000-test");
    await mkdir(releaseDir, { recursive: true });
    await symlink(releaseDir, paths.currentLink);

    const run: CommandRunner = async () => fail("Could not import vehicle_common.mode_loader");
    const result = await exportSchema(run, paths, "irrelevant.py");
    expect(result).toEqual({ success: false, error: "Could not import vehicle_common.mode_loader" });
  });

  it("surfaces a graceful error when the script produces non-JSON output", async () => {
    const releaseDir = join(scratchRoot, "releases", "20260101-000000-test");
    await mkdir(releaseDir, { recursive: true });
    await symlink(releaseDir, paths.currentLink);

    const run: CommandRunner = async () => ok("not json");
    const result = await exportSchema(run, paths, "irrelevant.py");
    expect(result).toEqual({ success: false, error: "schema export produced invalid JSON output" });
  });
});
