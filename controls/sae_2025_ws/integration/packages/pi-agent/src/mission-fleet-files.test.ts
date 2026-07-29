import { mkdir, mkdtemp, rm, symlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { missionNames, readFleetFile, readMissionFile, writeFleetFile, writeMissionFile } from "./mission-fleet-files.js";

describe("mission-fleet-files (real filesystem, no current release deployed)", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-mission-fleet-files-test-"));
    paths = deployPaths(scratchRoot);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  it("reports 'No build deployed' for every operation when there's no current symlink", async () => {
    expect(await missionNames(paths)).toEqual({ success: false, missions: [], error: "No build deployed" });
    expect(await readMissionFile("patrol", paths)).toEqual({ success: false, error: "No build deployed" });
    expect(await readFleetFile("example_fleet", paths)).toEqual({ success: false, error: "No build deployed" });
  });
});

describe("mission-fleet-files (real filesystem, a real current release deployed)", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-mission-fleet-files-test-"));
    paths = deployPaths(scratchRoot);

    const releaseDir = join(scratchRoot, "releases", "20260101-000000-test");
    await mkdir(join(releaseDir, "install", "uav", "share", "uav", "missions"), { recursive: true });
    await mkdir(join(releaseDir, "install", "uav", "share", "uav", "fleets"), { recursive: true });
    await mkdir(join(releaseDir, "install", "payload", "share", "payload", "missions"), { recursive: true });
    await writeFile(
      join(releaseDir, "install", "uav", "share", "uav", "missions", "patrol.yaml"),
      "modes:\n  start:\n    mode: uav.LandingMode\n",
      "utf-8",
    );
    await writeFile(
      join(releaseDir, "install", "payload", "share", "payload", "missions", "corner_navigate.yaml"),
      "modes: {}\n",
      "utf-8",
    );
    await writeFile(
      join(releaseDir, "install", "uav", "share", "uav", "fleets", "example_fleet.yaml"),
      "backend:\n  kind: hardware\nvehicles: []\n",
      "utf-8",
    );
    await mkdir(scratchRoot, { recursive: true });
    await symlink(releaseDir, paths.currentLink);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  it("lists mission names from both the uav and payload share dirs, sorted", async () => {
    const result = await missionNames(paths);
    expect(result).toEqual({ success: true, missions: ["corner_navigate", "patrol"] });
  });

  it("reads an existing mission file's real content", async () => {
    const result = await readMissionFile("patrol", paths);
    expect(result.success).toBe(true);
    expect(result.content).toContain("uav.LandingMode");
  });

  it("returns a clear error for a mission that doesn't exist", async () => {
    expect(await readMissionFile("nonexistent", paths)).toEqual({
      success: false,
      error: "Mission 'nonexistent' not found",
    });
  });

  it("rejects a mission name containing path traversal characters, never touching the filesystem", async () => {
    expect(await readMissionFile("../../../etc/passwd", paths)).toEqual({
      success: false,
      error: "Invalid mission name",
    });
    expect(await writeMissionFile("../evil", "haha", paths)).toEqual({
      success: false,
      error: "Invalid mission name",
    });
  });

  it("writes back to an existing mission file, and the write is genuinely persisted", async () => {
    const write = await writeMissionFile("patrol", "modes:\n  start:\n    mode: uav.NavGPSMode\n", paths);
    expect(write).toEqual({ success: true });

    const reread = await readMissionFile("patrol", paths);
    expect(reread.content).toContain("uav.NavGPSMode");
  });

  it("refuses to write a mission file that doesn't exist yet", async () => {
    expect(await writeMissionFile("brand_new_mission", "modes: {}", paths)).toEqual({
      success: false,
      error: "Mission 'brand_new_mission' not found",
    });
  });

  it("reads and writes an existing fleet file", async () => {
    const read = await readFleetFile("example_fleet", paths);
    expect(read.success).toBe(true);
    expect(read.content).toContain("hardware");

    const write = await writeFleetFile("example_fleet", "backend:\n  kind: hardware\nvehicles:\n  - name: air-01\n", paths);
    expect(write).toEqual({ success: true });

    const reread = await readFleetFile("example_fleet", paths);
    expect(reread.content).toContain("air-01");
  });

  it("rejects a fleet file name containing path traversal characters", async () => {
    expect(await readFleetFile("../../secrets", paths)).toEqual({ success: false, error: "Invalid fleet file name" });
  });
});
