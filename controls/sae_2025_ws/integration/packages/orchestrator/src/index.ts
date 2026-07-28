import Fastify from "fastify";
import websocketPlugin from "@fastify/websocket";
import WebSocket from "ws";
import { discoverPis } from "@pennair/integration-core";
import { PiAgentClient } from "./pi-agent-client.js";
import { startMission, triggerFailsafe } from "./mission.js";
import { attachHeartbeat } from "./heartbeat.js";

function piAgentPort(): number {
  return Number(process.env.PI_AGENT_PORT ?? 8090);
}

function piAgentClientFor(hostname: string): PiAgentClient {
  return new PiAgentClient({ hostname, port: piAgentPort() });
}

export function buildServer() {
  const app = Fastify();
  app.register(websocketPlugin);

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

  // Process lifecycle -- relayed to pi-agent, which runs systemctl locally.
  app.get<{ Querystring: { hostname?: string } }>("/api/mission/status", async (request, reply) => {
    const { hostname } = request.query;
    if (!hostname) {
      reply.code(400);
      return { success: false, running: false, state: "error", error: "hostname is required" };
    }
    return piAgentClientFor(hostname).missionStatus();
  });

  app.post<{ Body: { hostname?: string } }>("/api/mission/prepare", async (request, reply) => {
    const { hostname } = request.body;
    if (!hostname) {
      reply.code(400);
      return { success: false, error: "hostname is required" };
    }
    return piAgentClientFor(hostname).prepareMission();
  });

  app.post<{ Body: { hostname?: string } }>("/api/mission/stop", async (request, reply) => {
    const { hostname } = request.body;
    if (!hostname) {
      reply.code(400);
      return { success: false, error: "hostname is required" };
    }
    return piAgentClientFor(hostname).stopMission();
  });

  // In-graph mission RPC -- the orchestrator is itself a ROS2 participant
  // (see ros/trigger-service-client.ts), so these reach the vehicle's
  // mode_manager node directly over the network, not through pi-agent.
  app.post<{ Body: { vehicleName?: string } }>("/api/mission/start", async (request, reply) => {
    const { vehicleName } = request.body;
    if (!vehicleName) {
      reply.code(400);
      return { success: false, error: "vehicleName is required" };
    }
    return startMission(vehicleName);
  });

  app.post<{ Body: { vehicleName?: string } }>("/api/mission/failsafe", async (request, reply) => {
    const { vehicleName } = request.body;
    if (!vehicleName) {
      reply.code(400);
      return { success: false, error: "vehicleName is required" };
    }
    return triggerFailsafe(vehicleName);
  });

  // Real-time log relay: client <-> orchestrator <-> pi-agent, all real
  // WebSocket streaming, no polling anywhere in the chain.
  app.register(async (instance) => {
    instance.get<{ Querystring: { hostname?: string } }>(
      "/ws/mission/logs",
      { websocket: true },
      (clientSocket, request) => {
        const { hostname } = request.query;
        if (!hostname) {
          clientSocket.send(JSON.stringify({ type: "error", message: "hostname is required" }));
          clientSocket.close();
          return;
        }

        attachHeartbeat(clientSocket);
        const upstream = new WebSocket(`ws://${hostname}:${piAgentPort()}/ws/mission/logs`);

        upstream.on("message", (data) => {
          clientSocket.send(data.toString());
        });
        upstream.on("error", (error) => {
          clientSocket.send(JSON.stringify({ type: "error", message: error.message }));
          clientSocket.close();
        });
        upstream.on("close", () => clientSocket.close());

        clientSocket.on("close", () => upstream.close());
      },
    );
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
