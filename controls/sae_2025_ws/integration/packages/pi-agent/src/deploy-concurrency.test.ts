import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import * as tar from "tar";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { deployPaths, type DeployPaths } from "./deploy-paths.js";
import { pathExists, resolveSymlinkTarget } from "./deploy-fs.js";
import { extractRelease } from "./deploy.js";

/** Builds a real .tar.gz fixture, same helper shape as deploy.test.ts. */
async function buildFixtureArtifact(dir: string, name: string, marker: string): Promise<string> {
  const stagingDir = join(dir, `staging-${marker}`);
  await mkdir(join(stagingDir, "install"), { recursive: true });
  await writeFile(join(stagingDir, "install", "marker.txt"), marker, "utf-8");
  const artifactPath = join(dir, name);
  await tar.c({ file: artifactPath, cwd: stagingDir }, ["install"]);
  return artifactPath;
}

/**
 * Regression test for a concurrency bug that used to exist in extractRelease
 * (packages/pi-agent/src/deploy.ts): nothing serialized overlapping deploy
 * requests, so `symlinkForce`'s non-atomic rm-then-symlink and the
 * independent-per-call "prune stale releases" pass could corrupt release
 * state under concurrent calls (EEXIST crashes, a dangling `current` symlink,
 * or losing a release directory entirely). Fixed by serializing
 * extractRelease/activateRelease/rollbackRelease/deployArtifact through a
 * single in-process mutex (`withDeployLock` in deploy.ts) -- a single pi-agent
 * process only ever manages one deploy root, so this doesn't need to be a
 * cross-process/file lock.
 */
describe("extractRelease concurrency (packages/pi-agent/src/deploy.ts, deploy-fs.ts)", () => {
  let scratchRoot: string;
  let paths: DeployPaths;

  beforeEach(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-deploy-race-test-"));
    paths = deployPaths(scratchRoot);
  });

  afterEach(async () => {
    await rm(scratchRoot, { recursive: true, force: true });
  });

  it("two concurrent first-time deploys never corrupt release state (no thrown EEXIST, no dangling current symlink, no lost release)", async () => {
    // Run several trials with fresh scratch dirs -- before the deploy lock
    // existed, this reproduced on effectively every attempt on a real
    // filesystem (confirmed manually: 8/8 trials outside vitest). We only
    // need one trial to show corruption for the test to fail.
    let reproduced = false;
    let lastDetail = "";

    for (let trial = 0; trial < 5 && !reproduced; trial++) {
      const trialRoot = await mkdtemp(join(tmpdir(), "pi-agent-deploy-race-trial-"));
      const trialPaths = deployPaths(trialRoot);
      const now = () => Date.now(); // both calls share "the same second"

      const artifactA = await buildFixtureArtifact(trialRoot, "a.tar.gz", "A");
      const artifactB = await buildFixtureArtifact(trialRoot, "b.tar.gz", "B");

      const settled = await Promise.allSettled([
        extractRelease(artifactA, "a.tar.gz", trialPaths, now),
        extractRelease(artifactB, "b.tar.gz", trialPaths, now),
      ]);

      const rejected = settled.filter((s): s is PromiseRejectedResult => s.status === "rejected");
      if (rejected.length > 0) {
        reproduced = true;
        lastDetail = `a concurrent extractRelease call threw: ${rejected[0].reason}`;
      } else {
        const fulfilled = settled as PromiseFulfilledResult<Awaited<ReturnType<typeof extractRelease>>>[];
        const currentTarget = await resolveSymlinkTarget(trialPaths.currentLink);
        const currentExists = currentTarget ? await pathExists(currentTarget) : false;
        const releaseDirsStillExist = await Promise.all(fulfilled.map((f) => pathExists(f.value.releaseDir)));

        if ((currentTarget && !currentExists) || releaseDirsStillExist.some((exists) => !exists)) {
          reproduced = true;
          lastDetail = `currentTarget=${currentTarget} currentExists=${currentExists} releaseDirsStillExist=${releaseDirsStillExist.join(",")}`;
        }
      }

      await rm(trialRoot, { recursive: true, force: true });
    }

    expect(reproduced, `expected no corruption from concurrent deploys, but observed: ${lastDetail}`).toBe(false);
  });

  it("two sequential deploys within the same second, with the same artifact filename, get distinct releaseIds and a real previous release", async () => {
    // releaseTimestamp() (deploy.ts:27-29) truncates to whole seconds, so two
    // deploys of an identically-named artifact inside the same second (a very
    // plausible double-submit, e.g. an operator double-clicking "Deploy", or a
    // client retry after a slow-but-successful response) would collide on
    // releaseId without a disambiguating suffix. extractRelease appends a
    // random suffix specifically so this can't happen -- verified here rather
    // than assumed.
    const fixedNow = () => new Date("2026-01-15T10:00:00.000Z").getTime();

    const artifactV1 = await buildFixtureArtifact(scratchRoot, "build.tar.gz", "v1");
    const first = await extractRelease(artifactV1, "build.tar.gz", paths, fixedNow);

    const artifactV2 = await buildFixtureArtifact(scratchRoot, "build.tar.gz", "v2");
    const second = await extractRelease(artifactV2, "build.tar.gz", paths, fixedNow);

    // Distinct releaseId/releaseDir despite the identical timestamp+filename.
    expect(second.releaseId).not.toBe(first.releaseId);
    expect(second.releaseDir).not.toBe(first.releaseDir);

    // `previous` correctly points at the first release, not at the same
    // directory as `current` -- rollbackRelease has a real release to revert to.
    const currentTarget = await resolveSymlinkTarget(paths.currentLink);
    const previousTarget = await resolveSymlinkTarget(paths.previousLink);
    expect(currentTarget).toBe(second.releaseDir);
    expect(previousTarget).toBe(first.releaseDir);
    expect(previousTarget).not.toBe(currentTarget);
  });
});
