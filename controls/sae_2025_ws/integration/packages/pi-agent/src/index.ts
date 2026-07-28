import { createWriteStream } from "node:fs";
import { mkdir } from "node:fs/promises";
import { pipeline } from "node:stream/promises";
import Fastify from "fastify";
import websocketPlugin from "@fastify/websocket";
import multipartPlugin from "@fastify/multipart";
import { wifiConnect, wifiHotspot, wifiScan, wifiStatus } from "./wifi.js";
import { missionStatus, prepareMission, stopMission } from "./mission.js";
import { streamMissionLogs } from "./logs.js";
import { attachHeartbeat } from "./heartbeat.js";
import { currentBuild, deployArtifact, rollbackRelease } from "./deploy.js";
import { deployPaths } from "./deploy-paths.js";

export function buildServer() {
  const app = Fastify();
  app.register(websocketPlugin);
  app.register(multipartPlugin, {
    limits: { fileSize: 2 * 1024 * 1024 * 1024 }, // 2GB -- ROS2 build artifacts can be large
  });

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

  app.get("/api/deploy/current", async () => currentBuild());
  app.post("/api/deploy/rollback", async () => rollbackRelease());

  app.post("/api/deploy/upload", async (request, reply) => {
    const paths = deployPaths();
    await mkdir(paths.incomingDir, { recursive: true });

    let artifactPath: string | undefined;
    let sourceType = "";
    let sourceLabel = "";
    let vehicleName: string | undefined;
    let fleetFile: string | undefined;

    for await (const part of request.parts()) {
      if (part.type === "file") {
        artifactPath = `${paths.incomingDir}/${part.filename}`;
        await pipeline(part.file, createWriteStream(artifactPath));
      } else {
        const value = String(part.value ?? "");
        if (part.fieldname === "sourceType") sourceType = value;
        else if (part.fieldname === "sourceLabel") sourceLabel = value;
        else if (part.fieldname === "vehicleName") vehicleName = value || undefined;
        else if (part.fieldname === "fleetFile") fleetFile = value || undefined;
      }
    }

    if (!artifactPath) {
      reply.code(400);
      return { success: false, error: "No file uploaded" };
    }

    const filename = artifactPath.split("/").pop() ?? "artifact.tar.gz";
    return deployArtifact({
      artifactPath,
      artifactName: filename,
      sourceType: sourceType || "unknown",
      sourceLabel: sourceLabel || filename,
      vehicleName,
      fleetFile,
    });
  });

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
