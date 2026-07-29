import { createWriteStream } from "node:fs";
import { mkdir, stat } from "node:fs/promises";
import { homedir } from "node:os";
import { join } from "node:path";
import { pipeline } from "node:stream/promises";
import Fastify from "fastify";
import websocketPlugin from "@fastify/websocket";
import multipartPlugin from "@fastify/multipart";
import WebSocket from "ws";
import { discoverPis } from "@pennair/integration-core";
import { PiAgentClient } from "./pi-agent-client.js";
import { startMission, triggerFailsafe } from "./mission.js";
import { attachHeartbeat } from "./heartbeat.js";
import { BuildSourceStore } from "./build-source-store.js";
import { listBuilds } from "./github-builds.js";
import { resolveFleetCatalog } from "./fleet-catalog.js";

function piAgentPort(): number {
  return Number(process.env.PI_AGENT_PORT ?? 8090);
}

function piAgentClientFor(hostname: string): PiAgentClient {
  return new PiAgentClient({ hostname, port: piAgentPort() });
}

function orchestratorStateDir(): string {
  return process.env.ORCHESTRATOR_STATE_DIR ?? join(homedir(), ".pennair-orchestrator");
}

function githubRepo(): string {
  return process.env.GITHUB_REPO ?? "";
}

function githubToken(): string | undefined {
  return process.env.GITHUB_TOKEN || undefined;
}

function truthyQueryParam(value: unknown): boolean {
  return value === "true" || value === "1";
}

export function buildServer() {
  const app = Fastify();
  app.register(websocketPlugin);
  app.register(multipartPlugin, {
    limits: { fileSize: 2 * 1024 * 1024 * 1024 }, // 2GB -- ROS2 build artifacts can be large
  });

  const buildSourceStore = new BuildSourceStore(join(orchestratorStateDir(), "build-source.json"));
  let storeLoaded: Promise<void> | null = null;
  function ensureStoreLoaded(): Promise<void> {
    if (!storeLoaded) storeLoaded = buildSourceStore.load();
    return storeLoaded;
  }

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

  // -- Build source selection & GitHub build listing -----------------------
  app.get<{
    Querystring: {
      artifactPage?: string;
      artifactPageSize?: string;
      q?: string;
      sha?: string;
      commitSubject?: string;
      branch?: string;
      includeFallback?: string;
    };
  }>("/api/deploy/builds", async (request) => {
    const query = request.query;
    return listBuilds({
      githubRepo: githubRepo(),
      githubToken: githubToken(),
      artifactPage: query.artifactPage ? Number(query.artifactPage) : undefined,
      artifactPageSize: query.artifactPageSize ? Number(query.artifactPageSize) : undefined,
      q: query.q,
      sha: query.sha,
      commitSubject: query.commitSubject,
      branch: query.branch,
      includeFallback: truthyQueryParam(query.includeFallback),
    });
  });

  app.get("/api/deploy/source", async () => {
    await ensureStoreLoaded();
    return buildSourceStore.get();
  });

  app.post<{ Body: Record<string, unknown> }>("/api/deploy/source/github", async (request) => {
    await ensureStoreLoaded();
    const body = request.body;
    await buildSourceStore.setGithub({
      githubSource: body.githubSource as "release" | "actions",
      tag: body.tag as string | undefined,
      artifactId: body.artifactId as string | undefined,
      runId: body.runId as string | undefined,
      sha: body.sha as string | undefined,
      commitSubject: body.commitSubject as string | undefined,
      name: body.name as string | undefined,
      date: body.date as string | undefined,
      downloadUrl: body.downloadUrl as string | undefined,
      sizeMb: body.sizeMb as number | undefined,
      branch: body.branch as string | undefined,
      artifactName: body.artifactName as string | undefined,
      workflowName: body.workflowName as string | undefined,
      workflowEvent: body.workflowEvent as string | undefined,
      workflowConclusion: body.workflowConclusion as string | undefined,
    });
    return buildSourceStore.get();
  });

  app.post<{ Body: { codebaseRoot?: string } }>("/api/deploy/source/local-codebase", async (request, reply) => {
    await ensureStoreLoaded();
    const { codebaseRoot } = request.body;
    if (!codebaseRoot) {
      reply.code(400);
      return { success: false, error: "codebaseRoot is required" };
    }
    await buildSourceStore.setLocalCodebase(codebaseRoot);
    return buildSourceStore.get();
  });

  app.post<{ Body: { fleetFile?: string } }>("/api/deploy/source/fleet-file", async (request, reply) => {
    await ensureStoreLoaded();
    const { fleetFile } = request.body;
    if (!fleetFile) {
      reply.code(400);
      return { success: false, error: "fleetFile is required" };
    }
    await buildSourceStore.setFleetFile(fleetFile);
    return buildSourceStore.get();
  });

  app.post("/api/deploy/source/clear", async () => {
    await ensureStoreLoaded();
    await buildSourceStore.clear();
    return buildSourceStore.get();
  });

  // Local artifact: the operator uploads a build tarball to the orchestrator
  // itself (not to a specific Pi yet) so it becomes the selected build
  // source, the same two-step flow as the old dashboard (select a source,
  // then deploy it to whichever Pi later).
  app.post("/api/deploy/source/local-artifact", async (request, reply) => {
    await ensureStoreLoaded();
    const artifactDir = join(orchestratorStateDir(), "local-artifacts");
    await mkdir(artifactDir, { recursive: true });

    let savedPath: string | undefined;
    let artifactName: string | undefined;
    for await (const part of request.parts()) {
      if (part.type === "file") {
        artifactName = part.filename;
        savedPath = join(artifactDir, part.filename);
        await pipeline(part.file, createWriteStream(savedPath));
      }
    }

    if (!savedPath || !artifactName) {
      reply.code(400);
      return { success: false, error: "No file uploaded" };
    }

    const stats = await stat(savedPath);
    await buildSourceStore.setLocalArtifact(artifactName, savedPath, stats.size);
    return buildSourceStore.get();
  });

  app.get("/api/deploy/fleet-catalog", async () => {
    await ensureStoreLoaded();
    return resolveFleetCatalog({
      record: buildSourceStore.get(),
      cacheRoot: join(orchestratorStateDir(), "fleet-catalog-cache"),
      githubToken: githubToken(),
    });
  });

  // -- Per-Pi deploy relay ---------------------------------------------------
  app.get<{ Querystring: { hostname?: string } }>("/api/deploy/current", async (request, reply) => {
    const { hostname } = request.query;
    if (!hostname) {
      reply.code(400);
      return { installed: false, error: "hostname is required" };
    }
    return piAgentClientFor(hostname).currentBuild();
  });

  app.post<{ Body: { hostname?: string } }>("/api/deploy/rollback", async (request, reply) => {
    const { hostname } = request.body;
    if (!hostname) {
      reply.code(400);
      return { success: false, error: "hostname is required" };
    }
    return piAgentClientFor(hostname).rollbackDeploy();
  });

  app.post("/api/deploy/upload", async (request, reply) => {
    await ensureStoreLoaded();
    let hostname: string | undefined;
    let sourceType: string | undefined;
    let sourceLabel: string | undefined;
    let vehicleName: string | undefined;
    let fleetFile: string | undefined;

    for await (const part of request.parts()) {
      if (part.type === "file") {
        if (!hostname) {
          reply.code(400);
          return { success: false, error: "hostname is required" };
        }
        const client = piAgentClientFor(hostname);
        return client.uploadDeploy(part.filename, part.file, { sourceType, sourceLabel, vehicleName, fleetFile });
      }
      const value = String(part.value ?? "");
      if (part.fieldname === "hostname") hostname = value || undefined;
      else if (part.fieldname === "sourceType") sourceType = value || undefined;
      else if (part.fieldname === "sourceLabel") sourceLabel = value || undefined;
      else if (part.fieldname === "vehicleName") vehicleName = value || undefined;
      else if (part.fieldname === "fleetFile") fleetFile = value || undefined;
    }

    reply.code(400);
    return { success: false, error: "No file uploaded" };
  });

  // -- Mission/fleet YAML editors -- relayed to pi-agent, which reads/writes
  // files inside the currently-deployed release on that specific Pi (and
  // runs schema_export.py there, since the Pi is guaranteed to have a real
  // ROS2 + vehicle_common environment, unlike the orchestrator).
  app.get<{ Querystring: { hostname?: string } }>("/api/schema", async (request, reply) => {
    const { hostname } = request.query;
    if (!hostname) {
      reply.code(400);
      return { success: false, error: "hostname is required" };
    }
    return piAgentClientFor(hostname).schema();
  });

  app.get<{ Querystring: { hostname?: string } }>("/api/mission/mission-names", async (request, reply) => {
    const { hostname } = request.query;
    if (!hostname) {
      reply.code(400);
      return { success: false, missions: [], error: "hostname is required" };
    }
    return piAgentClientFor(hostname).missionNames();
  });

  app.get<{ Querystring: { hostname?: string; name?: string } }>("/api/mission/mission-file", async (request, reply) => {
    const { hostname, name } = request.query;
    if (!hostname || !name) {
      reply.code(400);
      return { success: false, error: "hostname and name are required" };
    }
    return piAgentClientFor(hostname).readMissionFile(name);
  });

  app.post<{ Body: { hostname?: string; name?: string; content?: string } }>(
    "/api/mission/mission-file",
    async (request, reply) => {
      const { hostname, name, content } = request.body;
      if (!hostname || !name) {
        reply.code(400);
        return { success: false, error: "hostname and name are required" };
      }
      return piAgentClientFor(hostname).writeMissionFile(name, content ?? "");
    },
  );

  app.get<{ Querystring: { hostname?: string; name?: string } }>("/api/fleet/fleet-file", async (request, reply) => {
    const { hostname, name } = request.query;
    if (!hostname || !name) {
      reply.code(400);
      return { success: false, error: "hostname and name are required" };
    }
    return piAgentClientFor(hostname).readFleetFile(name);
  });

  app.post<{ Body: { hostname?: string; name?: string; content?: string } }>(
    "/api/fleet/fleet-file",
    async (request, reply) => {
      const { hostname, name, content } = request.body;
      if (!hostname || !name) {
        reply.code(400);
        return { success: false, error: "hostname and name are required" };
      }
      return piAgentClientFor(hostname).writeFleetFile(name, content ?? "");
    },
  );

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
