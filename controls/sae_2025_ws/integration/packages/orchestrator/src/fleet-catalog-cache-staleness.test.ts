import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import * as tar from "tar";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import type { BuildSourceRecord } from "./build-source-store.js";
import { resolveFleetCatalog } from "./fleet-catalog.js";

async function buildArtifactWithFleet(dir: string, artifactPath: string, fleetName: string, marker: string) {
  const stagingDir = join(dir, `staging-${marker}`);
  await mkdir(join(stagingDir, "uav", "fleets"), { recursive: true });
  await writeFile(join(stagingDir, "uav", "fleets", fleetName), `marker: ${marker}`, "utf-8");
  await tar.c({ file: artifactPath, cwd: stagingDir, gzip: true }, ["uav"]);
}

/**
 * Regression test for a stale on-disk extraction cache in
 * resolveFleetCatalog (packages/orchestrator/src/fleet-catalog.ts).
 *
 * `catalogCacheKey` for a `local_artifact` record is derived only from
 * `record.localArtifactPath` -- the on-disk path the orchestrator saved the
 * uploaded file to. But that path is stable across repeated uploads of a
 * file with the same name: `POST /api/deploy/source/local-artifact`
 * (packages/orchestrator/src/index.ts) always saves to
 * `join(artifactDir, part.filename)` and overwrites whatever was already
 * there. Re-uploading a *different* local build under an unchanged filename
 * (an extremely normal workflow -- operators iterating locally tend to keep
 * re-exporting "build.tar.gz") used to reuse the exact same cache key and
 * extractDir, treated as still valid purely because the directory existed on
 * disk, regardless of whether the archive backing it had since changed.
 * Fixed by fingerprinting the artifact file (size+mtime) and re-extracting
 * whenever it no longer matches the fingerprint recorded at extraction time.
 */
describe("resolveFleetCatalog extraction cache for local_artifact", () => {
  let scratchDir: string;
  let cacheRoot: string;
  let artifactDir: string;

  beforeEach(async () => {
    scratchDir = await mkdtemp(join(tmpdir(), "fleet-catalog-stale-cache-test-"));
    cacheRoot = join(scratchDir, "cache");
    artifactDir = join(scratchDir, "local-artifacts");
    await mkdir(artifactDir, { recursive: true });
  });

  afterEach(async () => {
    await rm(scratchDir, { recursive: true, force: true });
  });

  it("keeps serving the first upload's fleet catalog after a same-named re-upload replaces the archive on disk", async () => {
    // Mirrors the real upload flow: the orchestrator always writes to
    // artifactDir/<filename>, overwriting on a repeat upload of the same name.
    const artifactPath = join(artifactDir, "build.tar.gz");

    await buildArtifactWithFleet(scratchDir, artifactPath, "example_fleet.yaml", "v1");
    const record: BuildSourceRecord = {
      kind: "local_artifact",
      artifactName: "build.tar.gz",
      localArtifactPath: artifactPath,
    };

    const first = await resolveFleetCatalog({ record, cacheRoot });
    expect(first.error).toBeNull();
    const firstFleetPath = first.catalog["example_fleet.yaml"];
    expect(firstFleetPath).toBeTruthy();
    const { readFile } = await import("node:fs/promises");
    expect(await readFile(firstFleetPath, "utf-8")).toContain("v1");

    // Operator re-uploads a DIFFERENT build under the SAME filename -- the
    // real endpoint overwrites artifactPath in place, same as here.
    await buildArtifactWithFleet(scratchDir, artifactPath, "example_fleet.yaml", "v2-DIFFERENT-BUILD");

    const second = await resolveFleetCatalog({ record, cacheRoot });
    const secondFleetPath = second.catalog["example_fleet.yaml"];
    const secondContent = await readFile(secondFleetPath, "utf-8");

    expect(secondContent).toContain("v2-DIFFERENT-BUILD");
  });
});
