import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import * as tar from "tar";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import type { BuildSourceRecord } from "./build-source-store.js";
import { catalogYamlPaths, fleetCatalogLookupFromPaths, resolveFleetCatalog } from "./fleet-catalog.js";

describe("catalogYamlPaths / fleetCatalogLookupFromPaths", () => {
  let scratchDir: string;

  beforeEach(async () => {
    scratchDir = await mkdtemp(join(tmpdir(), "fleet-catalog-test-"));
  });

  afterEach(async () => {
    await rm(scratchDir, { recursive: true, force: true });
  });

  it("finds yaml files only under a 'fleets' path segment", async () => {
    await mkdir(join(scratchDir, "uav", "fleets"), { recursive: true });
    await mkdir(join(scratchDir, "uav", "missions"), { recursive: true });
    await writeFile(join(scratchDir, "uav", "fleets", "example_fleet.yaml"), "vehicles: []", "utf-8");
    await writeFile(join(scratchDir, "uav", "missions", "not_a_fleet.yaml"), "steps: []", "utf-8");

    const paths = await catalogYamlPaths(scratchDir);
    expect(paths).toHaveLength(1);
    expect(paths[0]).toMatch(/example_fleet\.yaml$/);
  });

  it("prefers the installed install/uav/share/uav/fleets copy over a bare uav/fleets copy", async () => {
    await mkdir(join(scratchDir, "install", "uav", "share", "uav", "fleets"), { recursive: true });
    await mkdir(join(scratchDir, "uav", "fleets"), { recursive: true });
    await writeFile(join(scratchDir, "install", "uav", "share", "uav", "fleets", "example_fleet.yaml"), "installed", "utf-8");
    await writeFile(join(scratchDir, "uav", "fleets", "example_fleet.yaml"), "bare", "utf-8");

    const paths = await catalogYamlPaths(scratchDir);
    const catalog = fleetCatalogLookupFromPaths(paths);
    expect(Object.keys(catalog)).toEqual(["example_fleet.yaml"]);
    expect(catalog["example_fleet.yaml"]).toContain(join("install", "uav", "share", "uav", "fleets"));
  });

  it("prefers src/uav/uav/fleets over a bare uav/fleets copy", async () => {
    await mkdir(join(scratchDir, "src", "uav", "uav", "fleets"), { recursive: true });
    await mkdir(join(scratchDir, "uav", "fleets"), { recursive: true });
    await writeFile(join(scratchDir, "src", "uav", "uav", "fleets", "example_fleet.yaml"), "src copy", "utf-8");
    await writeFile(join(scratchDir, "uav", "fleets", "example_fleet.yaml"), "bare copy", "utf-8");

    const catalog = fleetCatalogLookupFromPaths(await catalogYamlPaths(scratchDir));
    expect(catalog["example_fleet.yaml"]).toContain(join("src", "uav", "uav", "fleets"));
  });
});

describe("resolveFleetCatalog", () => {
  let scratchDir: string;
  let cacheRoot: string;

  beforeEach(async () => {
    scratchDir = await mkdtemp(join(tmpdir(), "fleet-catalog-resolve-test-"));
    cacheRoot = join(scratchDir, "cache");
  });

  afterEach(async () => {
    await rm(scratchDir, { recursive: true, force: true });
  });

  it("returns an empty catalog for kind:none", async () => {
    const result = await resolveFleetCatalog({ record: { kind: "none" }, cacheRoot });
    expect(result).toEqual({ catalog: {}, source: null, error: null });
  });

  it("reads directly from disk for kind:local_codebase", async () => {
    const codebaseRoot = join(scratchDir, "repo");
    await mkdir(join(codebaseRoot, "src", "uav", "uav", "fleets"), { recursive: true });
    await writeFile(join(codebaseRoot, "src", "uav", "uav", "fleets", "example_fleet.yaml"), "vehicles: []", "utf-8");

    const record: BuildSourceRecord = { kind: "local_codebase", codebaseRoot };
    const result = await resolveFleetCatalog({ record, cacheRoot });
    expect(result.error).toBeNull();
    expect(Object.keys(result.catalog)).toEqual(["example_fleet.yaml"]);
  });

  it("extracts a local .tar.gz artifact and caches the extraction on disk", async () => {
    const stagingDir = join(scratchDir, "staging");
    await mkdir(join(stagingDir, "install", "uav", "share", "uav", "fleets"), { recursive: true });
    await writeFile(join(stagingDir, "install", "uav", "share", "uav", "fleets", "example_fleet.yaml"), "vehicles: []", "utf-8");
    await writeFile(join(stagingDir, "install", "README.txt"), "not yaml", "utf-8");
    const artifactPath = join(scratchDir, "artifact.tar.gz");
    await tar.c({ file: artifactPath, cwd: stagingDir, gzip: true }, ["install"]);

    const record: BuildSourceRecord = { kind: "local_artifact", artifactName: "artifact.tar.gz", localArtifactPath: artifactPath };
    const first = await resolveFleetCatalog({ record, cacheRoot });
    expect(first.error).toBeNull();
    expect(Object.keys(first.catalog)).toEqual(["example_fleet.yaml"]);

    // Second call should reuse the already-extracted cache dir rather than re-extracting.
    const second = await resolveFleetCatalog({ record, cacheRoot });
    expect(second).toEqual(first);
  });

  it("reports a clear error when the selected local artifact no longer exists", async () => {
    const record: BuildSourceRecord = {
      kind: "local_artifact",
      artifactName: "gone.tar.gz",
      localArtifactPath: join(scratchDir, "gone.tar.gz"),
    };
    const result = await resolveFleetCatalog({ record, cacheRoot });
    expect(result.catalog).toEqual({});
    expect(result.error).toMatch(/no longer exists/);
  });

  it("downloads and extracts a github release archive via the injected fetch", async () => {
    const stagingDir = join(scratchDir, "staging");
    await mkdir(join(stagingDir, "uav", "fleets"), { recursive: true });
    await writeFile(join(stagingDir, "uav", "fleets", "example_fleet.yaml"), "vehicles: []", "utf-8");
    const artifactPath = join(scratchDir, "release.tar.gz");
    await tar.c({ file: artifactPath, cwd: stagingDir, gzip: true }, ["uav"]);
    const archiveBytes = await import("node:fs/promises").then((fs) => fs.readFile(artifactPath));

    let fetchCalls = 0;
    const fetchImpl = (async () => {
      fetchCalls += 1;
      return new Response(archiveBytes);
    }) as typeof fetch;

    const record: BuildSourceRecord = {
      kind: "github",
      githubSource: "release",
      tag: "build-7",
      downloadUrl: "https://example.com/build-7.tar.gz",
    };
    const result = await resolveFleetCatalog({ record, cacheRoot, fetchImpl });
    expect(result.error).toBeNull();
    expect(Object.keys(result.catalog)).toEqual(["example_fleet.yaml"]);
    expect(fetchCalls).toBe(1);

    await resolveFleetCatalog({ record, cacheRoot, fetchImpl });
    expect(fetchCalls).toBe(1); // second call reused the on-disk extract cache
  });

  it("explicitly refuses .zip github actions artifacts rather than silently returning empty", async () => {
    const record: BuildSourceRecord = { kind: "github", githubSource: "actions", artifactId: "55" };
    const result = await resolveFleetCatalog({ record, cacheRoot });
    expect(result.catalog).toEqual({});
    expect(result.error).toMatch(/aren't supported/);
  });

  it("errors when a github release source has no download URL", async () => {
    const record: BuildSourceRecord = { kind: "github", githubSource: "release", tag: "build-8" };
    const result = await resolveFleetCatalog({ record, cacheRoot });
    expect(result.catalog).toEqual({});
    expect(result.error).toMatch(/does not expose a download URL/);
  });
});
