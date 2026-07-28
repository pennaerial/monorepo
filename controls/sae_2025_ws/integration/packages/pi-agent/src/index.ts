import Fastify from "fastify";
import { wifiConnect, wifiHotspot, wifiScan, wifiStatus } from "./wifi.js";

export function buildServer() {
  const app = Fastify();

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
