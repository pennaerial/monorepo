import { OrchestratorClient } from "@pennair/integration-core";

// Expo's env var convention (process.env.EXPO_PUBLIC_*) instead of Vite's
// (import.meta.env.VITE_*) -- see packages/web/src/orchestratorClient.ts.
const ORCHESTRATOR_URL = process.env.EXPO_PUBLIC_ORCHESTRATOR_URL ?? "http://localhost:8080";

export const client = new OrchestratorClient({ baseUrl: ORCHESTRATOR_URL });
