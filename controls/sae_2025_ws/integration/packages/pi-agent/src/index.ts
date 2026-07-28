import Fastify from "fastify";
import websocketPlugin from "@fastify/websocket";
import { wifiConnect, wifiHotspot, wifiScan, wifiStatus } from "./wifi.js";
import { missionStatus, prepareMission, stopMission } from "./mission.js";
import { streamMissionLogs } from "./logs.js";
import { attachHeartbeat } from "./heartbeat.js";

export function buildServer() {
  const app = Fastify();
  app.register(websocketPlugin);

  app.get("/health", async () => ({ status: "ok" }));

  app.get("/api/wifi/status", async () => wifiStatus());
  app.get("/api/wifi/scan", async () => wifiScan());

  app.post<{ Body: { ssid: string; password?: string } }>(
    "/api/wifi/connect",
    async (request, reply) => {
      const { ssid, password } = request.body;
      if (!ssid) {
        reply.code(400);
        return { success: false, error: "ssid is required" };
      }
      return wifiConnect(ssid, password ?? "");
    },
  );

  app.post("/api/wifi/hotspot", async () => wifiHotspot());

  app.get("/api/mission/status", async () => missionStatus());
  app.post("/api/mission/prepare", async () => prepareMission());
  app.post("/api/mission/stop", async () => stopMission());

  app.register(async (instance) => {
    instance.get("/ws/mission/logs", { websocket: true }, (socket) => {
      attachHeartbeat(socket);
      streamMissionLogs(socket);
    });
  });

  return app;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const port = Number(process.env.PORT ?? 8090);
  const app = buildServer();
  app.listen({ port, host: "0.0.0.0" }).catch((error) => {
    app.log.error(error);
    process.exit(1);
  });
}
