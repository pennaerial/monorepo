import { mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { BuildSourceStore } from "./build-source-store.js";

describe("BuildSourceStore", () => {
  let scratchDir: string;
  let storePath: string;

  beforeEach(async () => {
    scratchDir = await mkdtemp(join(tmpdir(), "build-source-store-test-"));
    storePath = join(scratchDir, "build-source.json");
  });

  afterEach(async () => {
    await rm(scratchDir, { recursive: true, force: true });
  });

  it("defaults to kind:none with no file on disk", async () => {
    const store = new BuildSourceStore(storePath);
    await store.load();
    expect(store.get()).toEqual({ kind: "none", summary: "No build source selected" });
  });

  it("persists a github selection to disk and reloads it in a fresh instance", async () => {
    const store = new BuildSourceStore(storePath);
    await store.load();
    await store.setGithub({ githubSource: "release", tag: "build-42", name: "Build 42" });

    const reloaded = new BuildSourceStore(storePath);
    await reloaded.load();
    expect(reloaded.get().kind).toBe("github");
    expect(reloaded.get().tag).toBe("build-42");
    expect(reloaded.get().summary).toBe("Build 42");
  });

  it("carries the selected fleetFile over when switching to a new github source", async () => {
    const store = new BuildSourceStore(storePath);
    await store.load();
    await store.setFleetFile("example_fleet.yaml");
    await store.setGithub({ githubSource: "release", tag: "build-42" });

    expect(store.get().fleetFile).toBe("example_fleet.yaml");
  });

  it("clear() resets to kind:none", async () => {
    const store = new BuildSourceStore(storePath);
    await store.load();
    await store.setLocalArtifact("build.tar.gz", "/tmp/cache/build.tar.gz", 12345);
    await store.clear();

    expect(store.get()).toMatchObject({ kind: "none" });
  });

  it("summarizes github actions sources using name/artifactName/artifactId in priority order", async () => {
    const store = new BuildSourceStore(storePath);
    await store.load();
    await store.setGithub({ githubSource: "actions", artifactId: "999" });
    expect(store.get().summary).toBe("GitHub Actions 999");
  });
});
