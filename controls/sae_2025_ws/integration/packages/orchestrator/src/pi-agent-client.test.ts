import { createReadStream, existsSync } from "node:fs";
import { mkdir, mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import * as tar from "tar";
import { afterAll, afterEach, beforeAll, describe, expect, it } from "vitest";
import { buildServer as buildPiAgentServer } from "@pennair/pi-agent";
import { PiAgentClient } from "./pi-agent-client.js";

describe("PiAgentClient against a real local pi-agent instance", () => {
  const piAgent = buildPiAgentServer();
  let port: number;

  beforeAll(async () => {
    await piAgent.listen({ port: 0, host: "127.0.0.1" });
    const address = piAgent.server.address();
    if (typeof address === "string" || address === null) {
      throw new Error("expected pi-agent to listen on a TCP port");
    }
    port = address.port;
  });

  afterAll(async () => {
    await piAgent.close();
  });

  it("reports healthy when pi-agent is reachable", async () => {
    const client = new PiAgentClient({ hostname: "127.0.0.1", port });
    expect(await client.isHealthy()).toBe(true);
  });

  it("reports unhealthy when nothing is listening on the port", async () => {
    const client = new PiAgentClient({ hostname: "127.0.0.1", port: 1 });
    expect(await client.isHealthy()).toBe(false);
  });

  it("relays missionStatus to the real pi-agent (no systemctl on this dev machine, so a graceful error is the correct real response)", async () => {
    const client = new PiAgentClient({ hostname: "127.0.0.1", port });
    const status = await client.missionStatus();
    // As with wifi: not asserting success:true, there's no real systemd here.
    // What matters is the request actually reached pi-agent's real handler
    // and got a real (non-hung, non-crashed) response back.
    expect(status).toHaveProperty("success");
    expect(status).toHaveProperty("state");
  });
});

describe("PiAgentClient deploy relay, against a real local pi-agent instance with a scratch DEPLOY_ROOT", () => {
  let scratchRoot: string;
  let piAgent: ReturnType<typeof buildPiAgentServer>;
  let port: number;
  const originalDeployRoot = process.env.DEPLOY_ROOT;

  beforeAll(async () => {
    scratchRoot = await mkdtemp(join(tmpdir(), "pi-agent-client-deploy-test-"));
    process.env.DEPLOY_ROOT = scratchRoot;
    piAgent = buildPiAgentServer();
    await piAgent.listen({ port: 0, host: "127.0.0.1" });
    const address = piAgent.server.address();
    if (typeof address === "string" || address === null) {
      throw new Error("expected pi-agent to listen on a TCP port");
    }
    port = address.port;
  });

  afterAll(async () => {
    await piAgent.close();
    process.env.DEPLOY_ROOT = originalDeployRoot;
    await rm(scratchRoot, { recursive: true, force: true });
  });

  afterEach(async () => {
    await rm(`${scratchRoot}/releases`, { recursive: true, force: true });
    await rm(`${scratchRoot}/current`, { force: true });
    await rm(`${scratchRoot}/previous`, { force: true });
  });

  it("relays currentBuild when nothing has been deployed yet", async () => {
    const client = new PiAgentClient({ hostname: "127.0.0.1", port });
    const result = await client.currentBuild();
    expect(result.installed).toBe(false);
  });

  it("relays a real streamed multipart upload through to pi-agent, which genuinely extracts it", async () => {
    const stagingDir = join(scratchRoot, "staging");
    await mkdir(join(stagingDir, "install"), { recursive: true });
    await writeFile(join(stagingDir, "install", "marker.txt"), "v1", "utf-8");
    const artifactPath = join(scratchRoot, "relay-test.tar.gz");
    await tar.c({ file: artifactPath, cwd: stagingDir, gzip: true }, ["install"]);

    const client = new PiAgentClient({ hostname: "127.0.0.1", port });
    const result = await client.uploadDeploy("relay-test.tar.gz", createReadStream(artifactPath), {
      sourceType: "github",
      sourceLabel: "build-99",
      vehicleName: "air-01",
    });

    // Real graceful failure expected: no real systemd/sudo on this dev machine.
    expect(result.success).toBe(false);
    expect(result.releaseId).toBeTruthy();

    expect(existsSync(`${scratchRoot}/current/install/marker.txt`)).toBe(true);
  });
});
