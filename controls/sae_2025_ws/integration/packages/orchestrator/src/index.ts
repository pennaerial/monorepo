import Fastify from "fastify";

export function buildServer() {
  const app = Fastify();

  app.get("/health", async () => ({ status: "ok" }));

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
