import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import * as tar from "tar";
import { afterAll, afterEach, beforeAll, describe, expect, it } from "vitest";
import { buildServer } from "./index.js";
import { pathExists } from "./deploy-fs.js";

describe("pi-agent server", () => {
  it("responds ok on /health", async () => {
    const app = buildServer();
    const response = await app.inject({ method: "GET", url: "/health" });
    expect(response.statusCode).toBe(200);
    expect(response.json()).toEqual({ status: "ok" });
  });
});

describe("pi-agent deploy upload endpoint, over a real HTTP multipart request", () => {
  let scratchRoot: string;
  let app: ReturnType<typeof buildServer>;
  let port: number;
  const originalDeployRoot = process.env.DEPLOY_ROOT;

  beforeAll(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-upload-test-"));
    process.env.DEPLOY_ROOT = scratchRoot;
    app = buildServer();
    await app.listen({ port: 0, host: "127.0.0.1" });
    const address = app.server.address();
    if (typeof address === "string" || address === null) {
      throw new Error("expected server to listen on a TCP port");
    }
    port = address.port;
  });

  afterAll(async () => {
    await app.close();
    process.env.DEPLOY_ROOT = originalDeployRoot;
    await rm(scratchRoot, { recursive: true, force: true });
  });

  afterEach(async () => {
    // deployArtifact's real (non-injected) CommandRunner will genuinely try
    // `sudo -n install ...`/`systemctl` on this dev machine and fail (no real
    // systemd here) -- that's the expected, correct real response, same
    // pattern as the wifi/mission relay tests. Clean the release dirs
    // between tests so each starts fresh.
    await rm(`${scratchRoot}/releases`, { recursive: true, force: true });
    await rm(`${scratchRoot}/current`, { force: true });
    await rm(`${scratchRoot}/previous`, { force: true });
  });

  it("accepts a real multipart upload, extracts it for real, and gracefully fails activation (no real systemd here)", async () => {
    const stagingDir = join(scratchRoot, "staging");
    await mkdir(join(stagingDir, "install"), { recursive: true });
    await writeFile(join(stagingDir, "install", "marker.txt"), "v1", "utf-8");
    const artifactPath = join(scratchRoot, "upload-test.tar.gz");
    await tar.c({ file: artifactPath, cwd: stagingDir }, ["install"]);

    const form = new FormData();
    form.set("sourceType", "github");
    form.set("sourceLabel", "build-42");
    form.set("vehicleName", "air-01");
    const fileBuffer = await import("node:fs/promises").then((fs) => fs.readFile(artifactPath));
    form.set("file", new Blob([fileBuffer]), "upload-test.tar.gz");

    const response = await fetch(`http://127.0.0.1:${port}/api/deploy/upload`, {
      method: "POST",
      body: form,
    });
    const body = (await response.json()) as { success: boolean; error?: string; releaseId?: string };

    // Real graceful failure expected: no real systemd/sudo on this dev machine.
    expect(body.success).toBe(false);
    expect(body.error).toBeTruthy();
    expect(body.releaseId).toBeTruthy(); // extraction + metadata write happened before activation failed

    // The real extraction genuinely happened, even though activation failed.
    expect(await pathExists(`${scratchRoot}/current/install/marker.txt`)).toBe(true);
  });

  it("returns 400 when no file is uploaded", async () => {
    const form = new FormData();
    form.set("sourceType", "github");

    const response = await fetch(`http://127.0.0.1:${port}/api/deploy/upload`, {
      method: "POST",
      body: form,
    });
    expect(response.status).toBe(400);
    expect((await response.json()) as { success: boolean }).toMatchObject({ success: false });
  });
});
