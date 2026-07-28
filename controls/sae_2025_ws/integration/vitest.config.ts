import { fileURLToPath } from "node:url";
import { defineConfig } from "vitest/config";

// Resolve workspace packages to their TS source rather than their built
// `dist/` output, so `vitest` never depends on packages having been built
// first -- regardless of whether it's invoked via `npm test` (which builds
// via the `pretest` hook) or directly via `npx vitest run`.
export default defineConfig({
  resolve: {
    alias: {
      "@pennair/integration-core": fileURLToPath(
        new URL("./packages/core/src/index.ts", import.meta.url),
      ),
      "@pennair/pi-agent": fileURLToPath(
        new URL("./packages/pi-agent/src/index.ts", import.meta.url),
      ),
    },
  },
  test: {
    include: ["packages/*/src/**/*.test.ts"],
  },
});
