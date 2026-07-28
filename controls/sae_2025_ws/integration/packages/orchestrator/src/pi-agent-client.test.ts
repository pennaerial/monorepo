import { afterAll, beforeAll, describe, expect, it } from "vitest";
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
});
