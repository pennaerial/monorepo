import { mkdir, mkdtemp, rm, symlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import * as tar from "tar";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import type { CommandRunner, ExecResult } from "./exec.js";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { pathExists, resolveSymlinkTarget } from "./deploy-fs.js";
import {
  activateRelease,
  currentBuild,
  deployArtifact,
  extractRelease,
  rollbackRelease,
  writeReleaseMetadata,
} from "./deploy.js";

function ok(stdout = ""): ExecResult {
  return { code: 0, stdout, stderr: "" };
}
function fail(stderr: string): ExecResult {
  return { code: 1, stdout: "", stderr };
}
const noWait = async () => {};

/** Builds a real .tar.gz fixture (using the same `tar` package deploy.ts
 *  uses to extract) containing an install/ dir, so extraction is exercised
 *  against a real archive, not a mock. */
async function buildFixtureArtifact(dir: string, name: string, marker: string): Promise<string> {
  const stagingDir = join(dir, "staging");
  await mkdir(join(stagingDir, "install"), { recursive: true });
  await writeFile(join(stagingDir, "install", "marker.txt"), marker, "utf-8");
  const artifactPath = join(dir, name);
  await tar.c({ file: artifactPath, cwd: stagingDir }, ["install"]);
  return artifactPath;
}

describe("deploy.ts (real filesystem, faked systemctl)", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-deploy-test-"));
    paths = deployPaths(scratchRoot);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  describe("extractRelease", () => {
    it("extracts a real tarball into a timestamped release dir and points current at it", async () => {
      const artifactPath = await buildFixtureArtifact(scratchRoot, "build.tar.gz", "v1");
      const now = () => new Date("2026-01-15T10:30:00Z").getTime();

      const result = await extractRelease(artifactPath, "build.tar.gz", paths, now);

      expect(result.releaseId).toBe("20260115-103000-build");
      expect(await pathExists(`${result.releaseDir}/install/marker.txt`)).toBe(true);
      expect(await resolveSymlinkTarget(paths.currentLink)).toBe(result.releaseDir);
      expect(await pathExists(artifactPath)).toBe(false); // artifact cleaned up after extraction
      expect(result.previousTarget).toBe(""); // nothing was current before
    });

    it("keeps the old current as previous, and prunes any other stale release dirs", async () => {
      const firstArtifact = await buildFixtureArtifact(scratchRoot, "first.tar.gz", "v1");
      const first = await extractRelease(firstArtifact, "first.tar.gz", paths, () =>
        new Date("2026-01-15T10:00:00Z").getTime(),
      );

      const secondArtifact = await buildFixtureArtifact(scratchRoot, "second.tar.gz", "v2");
      const second = await extractRelease(secondArtifact, "second.tar.gz", paths, () =>
        new Date("2026-01-15T11:00:00Z").getTime(),
      );

      expect(second.previousTarget).toBe(first.releaseDir);
      expect(await resolveSymlinkTarget(paths.currentLink)).toBe(second.releaseDir);
      expect(await resolveSymlinkTarget(paths.previousLink)).toBe(first.releaseDir);

      // A stray unrelated directory in releases/ should be pruned since it's
      // neither the new current nor the retained previous.
      const strayDir = `${paths.releasesDir}/some-stray-release`;
      await mkdir(strayDir, { recursive: true });
      const thirdArtifact = await buildFixtureArtifact(scratchRoot, "third.tar.gz", "v3");
      await extractRelease(thirdArtifact, "third.tar.gz", paths, () => new Date("2026-01-15T12:00:00Z").getTime());
      expect(await pathExists(strayDir)).toBe(false);
      // the now-two-releases-back "first" should also have been pruned
      expect(await pathExists(first.releaseDir)).toBe(false);
    });

    it("throws and cleans up if the archive has no install/ directory", async () => {
      const stagingDir = join(scratchRoot, "bad-staging");
      await mkdir(join(stagingDir, "not-install"), { recursive: true });
      await writeFile(join(stagingDir, "not-install", "f.txt"), "x", "utf-8");
      const artifactPath = join(scratchRoot, "bad.tar.gz");
      await tar.c({ file: artifactPath, cwd: stagingDir }, ["not-install"]);

      await expect(extractRelease(artifactPath, "bad.tar.gz", paths)).rejects.toThrow(/install/);
    });
  });

  describe("activateRelease", () => {
    it("succeeds once the service reports a stable active pid", async () => {
      const releaseDir = `${paths.releasesDir}/rel-1`;
      let call = 0;
      const run: CommandRunner = async (command, args) => {
        const key = [command, ...args].join(" ");
        if (key === "sudo -n systemctl restart pennair-autonomy.service") return ok();
        if (key.startsWith("sudo -n systemctl show")) {
          call += 1;
          return ok("active\nrunning\n4242\n0");
        }
        return fail(`unexpected: ${key}`);
      };
      let clock = 0;
      const now = () => clock;
      const wait = async () => {
        clock += 1000;
      };

      const result = await activateRelease(releaseDir, "", run, wait, now, paths);
      expect(result).toEqual({ success: true, output: "Release activated" });
      expect(call).toBeGreaterThanOrEqual(2); // needs at least 2 stable readings
    });

    it("rolls back to the previous release if the restart itself fails", async () => {
      const previousDir = join(scratchRoot, "releases", "prev-release");
      await mkdir(previousDir, { recursive: true });
      const failedDir = join(scratchRoot, "releases", "failed-release");
      await mkdir(failedDir, { recursive: true });

      const restartCalls: string[] = [];
      const run: CommandRunner = async (command, args) => {
        const key = [command, ...args].join(" ");
        restartCalls.push(key);
        if (key === "sudo -n systemctl restart pennair-autonomy.service") {
          return fail("Unit pennair-autonomy.service not found.");
        }
        return ok();
      };

      const result = await activateRelease(failedDir, previousDir, run, noWait, Date.now, paths);
      expect(result.success).toBe(false);
      expect(result.error).toContain("Reverted to the previous release");
      expect(await resolveSymlinkTarget(paths.currentLink)).toBe(previousDir);
      expect(await resolveSymlinkTarget(paths.previousLink)).toBe(failedDir);
    });

    it("rolls back if the service is unstable (pid keeps changing)", async () => {
      const previousDir = join(scratchRoot, "releases", "prev-release");
      await mkdir(previousDir, { recursive: true });
      const newDir = join(scratchRoot, "releases", "new-release");
      await mkdir(newDir, { recursive: true });

      let showCall = 0;
      const run: CommandRunner = async (command, args) => {
        const key = [command, ...args].join(" ");
        if (key === "sudo -n systemctl restart pennair-autonomy.service") return ok();
        if (key.startsWith("sudo -n systemctl show")) {
          showCall += 1;
          // pid flips every call -> never stable
          return ok(`active\nrunning\n${1000 + showCall}\n0`);
        }
        return ok();
      };
      let clock = 0;
      const wait = async () => {
        clock += 1000;
      };

      const result = await activateRelease(newDir, previousDir, run, wait, () => clock, paths);
      expect(result.success).toBe(false);
      expect(result.error).toContain("unstable");
      expect(await resolveSymlinkTarget(paths.currentLink)).toBe(previousDir);
    });

    it("fails without rollback if there is no previous release available", async () => {
      const releaseDir = join(scratchRoot, "releases", "only-release");
      await mkdir(releaseDir, { recursive: true });
      const run: CommandRunner = async () => fail("boom");

      const result = await activateRelease(releaseDir, "", run, noWait, Date.now, paths);
      expect(result).toEqual({ success: false, error: "boom" });
    });
  });

  describe("rollbackRelease", () => {
    it("swaps current/previous and reactivates", async () => {
      const oldDir = join(scratchRoot, "releases", "old");
      const newDir = join(scratchRoot, "releases", "new");
      await mkdir(oldDir, { recursive: true });
      await mkdir(newDir, { recursive: true });
      await mkdir(paths.releasesDir, { recursive: true });
      await symlink(newDir, paths.currentLink);
      await symlink(oldDir, paths.previousLink);

      const run: CommandRunner = async (command, args) => {
        const key = [command, ...args].join(" ");
        if (key === "sudo -n systemctl restart pennair-autonomy.service") return ok();
        if (key.startsWith("sudo -n systemctl show")) return ok("active\nrunning\n999\n0");
        return ok();
      };
      // activateRelease deliberately runs the full 8s polling window before
      // declaring success (matches the old bash) -- use a fake clock so this
      // test doesn't actually wait 8 real seconds.
      let clock = 0;
      const wait = async () => {
        clock += 1000;
      };

      const result = await rollbackRelease(run, wait, () => clock, paths);
      expect(result.success).toBe(true);
      expect(await resolveSymlinkTarget(paths.currentLink)).toBe(oldDir);
      expect(await resolveSymlinkTarget(paths.previousLink)).toBe(newDir);
    });

    it("reports an error when there is no previous release", async () => {
      const result = await rollbackRelease(async () => ok(), noWait, Date.now, paths);
      expect(result).toEqual({ success: false, error: "No previous release to roll back to" });
    });
  });

  describe("currentBuild + writeReleaseMetadata", () => {
    it("reports not installed when nothing has been deployed", async () => {
      expect(await currentBuild(paths)).toEqual({ success: true, installed: false, info: "No build deployed" });
    });

    it("reports installed with a metadata summary once a release is current", async () => {
      const releaseDir = join(scratchRoot, "releases", "rel-1");
      await mkdir(releaseDir, { recursive: true });
      await mkdir(paths.releasesDir, { recursive: true });
      await symlink(releaseDir, paths.currentLink);
      await writeReleaseMetadata(releaseDir, {
        releaseId: "rel-1",
        sourceType: "github",
        sourceLabel: "build-42",
        vehicleName: "air-01",
        deployedAtUtc: "2026-01-15T10:00:00Z",
      });

      const status = await currentBuild(paths);
      expect(status.success).toBe(true);
      expect(status.installed).toBe(true);
      expect(status.releaseId).toBe("rel-1");
      expect(status.info).toContain("Vehicle: air-01");
      expect(status.info).toContain("Source label: build-42");
    });
  });

  describe("deployArtifact (full orchestration)", () => {
    it("extracts, installs the service, writes metadata, and activates -- end to end", async () => {
      const artifactPath = await buildFixtureArtifact(scratchRoot, "build.tar.gz", "v1");
      const run: CommandRunner = async (command, args) => {
        const key = [command, ...args].join(" ");
        if (key === "sudo -n systemctl restart pennair-autonomy.service") return ok();
        if (key.startsWith("sudo -n systemctl show")) return ok("active\nrunning\n123\n0");
        return ok(); // install/daemon-reload/enable all succeed
      };
      let clock = 0;
      const wait = async () => {
        clock += 1000;
      };

      const result = await deployArtifact(
        {
          artifactPath,
          artifactName: "build.tar.gz",
          sourceType: "github",
          sourceLabel: "build-42",
          vehicleName: "air-01",
        },
        run,
        wait,
        () => clock,
        paths,
      );

      expect(result.success).toBe(true);
      expect(result.releaseId).toBeTruthy();

      const status = await currentBuild(paths);
      expect(status.installed).toBe(true);
      expect(status.info).toContain("Vehicle: air-01");

      // runner script + systemd unit were actually written to disk
      expect(await pathExists(paths.runnerScript)).toBe(true);
    });

    it("stops before activating if the systemd unit install fails", async () => {
      const artifactPath = await buildFixtureArtifact(scratchRoot, "build.tar.gz", "v1");
      const run: CommandRunner = async (command, args) => {
        const key = [command, ...args].join(" ");
        if (key.startsWith("sudo -n install")) return fail("sudo: a password is required");
        return ok();
      };

      const result = await deployArtifact(
        { artifactPath, artifactName: "build.tar.gz", sourceType: "github", sourceLabel: "build-42" },
        run,
        noWait,
        Date.now,
        paths,
      );

      expect(result.success).toBe(false);
      expect(result.error).toContain("password");
      // the release was still extracted and made current, just never activated
      expect(await pathExists(paths.currentLink)).toBe(true);
    });
  });
});
