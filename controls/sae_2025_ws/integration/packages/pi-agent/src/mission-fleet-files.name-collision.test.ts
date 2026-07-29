import { mkdir, mkdtemp, rm, symlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { missionNames, readMissionFile } from "./mission-fleet-files.js";

/**
 * Known, deliberately-deferred gap (Low severity, zero current impact): if a
 * mission name ever collides across the uav and payload packages,
 * mission-fleet-files.ts's findMissionFilePath checks the uav package's
 * missions dir before the payload package's, and missionNames() just pushes
 * each package's bare basenames into a flat array with no de-duplication or
 * package tagging. There's no collision in the repo today (verified: `comm
 * -12` between uav/missions and payload/missions basenames is empty), but
 * nothing prevents it, and the failure mode if it ever happens is silent and
 * confusing rather than a clear error. A real fix requires changing
 * missionNames()'s return shape from a flat string[] to target-qualified
 * entries (mirroring schema_export.py's own `_available_mission_files`,
 * which already carries a `target` per entry) and threading that through
 * pi-agent, the orchestrator relay, core's client, and MissionEditor.tsx --
 * deferred given it's unreachable with any mission files that exist today.
 *
 * - missionNames() returns the same name twice (once per package), which
 *   the web UI's mission <select> renders as two indistinguishable, textually
 *   identical option entries with the same `value`/`key` (MissionEditor.tsx
 *   line 145-149's `key={name}` on the mapped <option>).
 * - Selecting either one always resolves to the uav mission (uav is checked
 *   first in MISSION_PACKAGES); the payload mission of the same name becomes
 *   permanently unreachable/uneditable through this API, with no error or
 *   warning anywhere -- you'd just always be editing (and, on save,
 *   silently overwriting reads of) the wrong file if you meant the payload
 *   one.
 */
describe("mission-fleet-files: uav/payload mission-name collision handling", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-mission-collision-test-"));
    paths = deployPaths(scratchRoot);

    const releaseDir = join(scratchRoot, "releases", "20260101-000000-test");
    await mkdir(join(releaseDir, "install", "uav", "share", "uav", "missions"), { recursive: true });
    await mkdir(join(releaseDir, "install", "payload", "share", "payload", "missions"), { recursive: true });
    await writeFile(
      join(releaseDir, "install", "uav", "share", "uav", "missions", "patrol.yaml"),
      "modes:\n  start:\n    mode: uav.LandingMode\n",
      "utf-8",
    );
    await writeFile(
      join(releaseDir, "install", "payload", "share", "payload", "missions", "patrol.yaml"),
      "modes:\n  start:\n    mode: payload.PayloadWaitForDriveOutMode\n",
      "utf-8",
    );
    await mkdir(scratchRoot, { recursive: true });
    await symlink(releaseDir, paths.currentLink);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  it("lists the colliding name twice with no way to tell the two apart", async () => {
    const result = await missionNames(paths);
    expect(result.success).toBe(true);
    // Bug: "patrol" appears twice, indistinguishably -- a client has no way
    // to know one instance means the uav mission and the other the payload
    // one.
    expect(result.missions).toEqual(["patrol", "patrol"]);
  });

  it("always resolves the colliding name to the uav file, silently hiding the payload one", async () => {
    const result = await readMissionFile("patrol", paths);
    expect(result.success).toBe(true);
    // The payload/missions/patrol.yaml content is unreachable via this name.
    expect(result.content).toContain("uav.LandingMode");
    expect(result.content).not.toContain("payload.PayloadWaitForDriveOutMode");
  });
});
