import Fastify from "fastify";
import { discoverPis } from "@pennair/integration-core";
import { PiAgentClient } from "./pi-agent-client.js";

function piAgentPort(): number {
  return Number(process.env.PI_AGENT_PORT ?? 8090);
}

function piAgentClientFor(hostname: string): PiAgentClient {
  return new PiAgentClient({ hostname, port: piAgentPort() });
}

export function buildServer() {
  const app = Fastify();

  app.get("/health", async () => ({ status: "ok" }));

  app.get("/api/discovery", async () => {
    const prefixes = (process.env.DISCOVERY_PREFIXES ?? "air,payload").split(",");
    const suffixMax = Number(process.env.DISCOVERY_SUFFIX_MAX ?? 20);
    const timeoutMs = Number(process.env.DISCOVERY_TIMEOUT_MS ?? 1000);
    const suffixes = Array.from({ length: suffixMax }, (_, i) => i + 1);
    const pis = await discoverPis({
      prefixes,
      suffixes,
      port: piAgentPort(),
      timeoutMs,
    });
    return { pis };
  });

  app.get<{ Querystring: { hostname?: string } }>("/api/wifi/status", async (request, reply) => {
    const { hostname } = request.query;
    if (!hostname) {
      reply.code(400);
      return { success: false, connections: [], error: "hostname is required" };
    }
    return piAgentClientFor(hostname).wifiStatus();
  });

  app.get<{ Querystring: { hostname?: string } }>("/api/wifi/scan", async (request, reply) => {
    const { hostname } = request.query;
    if (!hostname) {
      reply.code(400);
      return { success: false, networks: [], error: "hostname is required" };
    }
    return piAgentClientFor(hostname).wifiScan();
  });

  app.post<{ Body: { hostname?: string; ssid: string; password?: string } }>(
    "/api/wifi/connect",
    async (request, reply) => {
      const { hostname, ssid, password } = request.body;
      if (!hostname || !ssid) {
        reply.code(400);
        return { success: false, error: "hostname and ssid are required" };
      }
      return piAgentClientFor(hostname).wifiConnect(ssid, password ?? "");
    },
  );

  app.post<{ Body: { hostname?: string } }>("/api/wifi/hotspot", async (request, reply) => {
    const { hostname } = request.body;
    if (!hostname) {
      reply.code(400);
      return { success: false, error: "hostname is required" };
    }
    return piAgentClientFor(hostname).wifiHotspot();
  });

  return app;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const port = Number(process.env.PORT ?? 8080);
  const app = buildServer();
  app.listen({ port, host: "0.0.0.0" }).catch((error) => {
    app.log.error(error);
    process.exit(1);
  });
}
