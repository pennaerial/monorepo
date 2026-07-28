export interface PiAgentClientOptions {
  hostname: string;
  port: number;
  fetchImpl?: typeof fetch;
}

/** Thin HTTP client the orchestrator uses to reach one Pi's pi-agent daemon. */
export class PiAgentClient {
  private readonly baseUrl: string;
  private readonly fetchImpl: typeof fetch;

  constructor(options: PiAgentClientOptions) {
    this.baseUrl = `http://${options.hostname}:${options.port}`;
    this.fetchImpl = options.fetchImpl ?? fetch;
  }

  async isHealthy(): Promise<boolean> {
    try {
      const response = await this.fetchImpl(`${this.baseUrl}/health`);
      return response.ok;
    } catch {
      return false;
    }
  }
}
