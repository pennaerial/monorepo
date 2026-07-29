import { randomBytes } from "node:crypto";
import { Readable } from "node:stream";
import { mkdtemp, readdir, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { afterAll, afterEach, beforeAll, describe, expect, it } from "vitest";
import { buildServer } from "./index.js";

/**
 * Regression test: pi-agent's /api/deploy/upload handler now wraps the
 * `request.parts()` loop in a try/catch and removes the partial file it had
 * already streamed into `incoming/` if the upload is interrupted (client
 * disconnect, network blip, timeout) before finishing -- previously that
 * partial file was left behind forever, since it's never referenced by
 * extractRelease's own cleanup (which only runs for a fully-received artifact
 * that actually made it to `deployArtifact`). Repeated interrupted uploads
 * (very plausible on a flaky field WiFi link to a Pi) would otherwise
 * accumulate orphaned multi-hundred-MB files in `incoming/`.
 */
describe("pi-agent /api/deploy/upload cleans up a partial file after an aborted upload", () => {
  let scratchRoot: string;
  let app: ReturnType<typeof buildServer>;
  let port: number;
  const originalDeployRoot = process.env.DEPLOY_ROOT;

  beforeAll(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-abort-test-"));
    process.env.DEPLOY_ROOT = scratchRoot;
    app = buildServer();
    await app.listen({ port: 0, host: "127.0.0.1" });
    const address = app.server.address();
    if (typeof address === "string" || address === null) throw new Error("expected a TCP port");
    port = address.port;
  });

  afterAll(async () => {
    await app.close();
    process.env.DEPLOY_ROOT = originalDeployRoot;
    await rm(scratchRoot, { recursive: true, force: true });
  });

  afterEach(async () => {
    await rm(`${scratchRoot}/incoming`, { recursive: true, force: true });
  });

  it("does not leave a partial file in incoming/ after the client aborts mid-upload", async () => {
    const boundary = "----abort-test-boundary";
    async function* slowChunks() {
      for (let i = 0; i < 200; i++) {
        yield randomBytes(256 * 1024);
        await new Promise((resolve) => setTimeout(resolve, 15));
      }
    }
    async function* body() {
      const encoder = new TextEncoder();
      yield Buffer.from(
        encoder.encode(
          `--${boundary}\r\nContent-Disposition: form-data; name="file"; filename="aborted.tar.gz"\r\nContent-Type: application/octet-stream\r\n\r\n`,
        ),
      );
      for await (const chunk of slowChunks()) yield chunk;
      yield Buffer.from(encoder.encode(`\r\n--${boundary}--\r\n`));
    }

    const controller = new AbortController();
    setTimeout(() => controller.abort(), 200);

    await fetch(`http://127.0.0.1:${port}/api/deploy/upload`, {
      method: "POST",
      headers: { "Content-Type": `multipart/form-data; boundary=${boundary}` },
      body: Readable.from(body()) as unknown as ReadableStream,
      duplex: "half",
      signal: controller.signal,
    } as RequestInit).catch(() => {
      // Expected: the client-side fetch itself rejects once aborted.
    });

    await new Promise((resolve) => setTimeout(resolve, 500)); // let the server settle post-abort

    const incomingContents = await readdir(`${scratchRoot}/incoming`).catch(() => []);
    expect(incomingContents).toEqual([]);
  });
});
