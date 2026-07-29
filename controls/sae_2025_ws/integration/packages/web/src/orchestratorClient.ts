import { OrchestratorClient } from "@pennair/integration-core";

const ORCHESTRATOR_URL = import.meta.env.VITE_ORCHESTRATOR_URL ?? "http://localhost:8080";

export const client = new OrchestratorClient({ baseUrl: ORCHESTRATOR_URL });
