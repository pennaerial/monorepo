export interface TriggerResult {
  success: boolean;
  message: string;
}

/** Calls a `std_srvs/srv/Trigger` ROS2 service and returns its response. */
export type TriggerServiceCaller = (serviceName: string) => Promise<TriggerResult>;

interface RclNodeJsModule {
  init(): Promise<void>;
  createNode(name: string): RclNode;
  shutdown(): void;
}

interface RclNode {
  createClient(type: string, name: string): RclServiceClient;
  spin(): void;
}

interface RclServiceClient {
  waitForService(timeoutMs: number): Promise<boolean>;
  sendRequest(request: Record<string, never>): Promise<{ success: boolean; message: string }>;
}

let cachedNode: RclNode | null = null;
const cachedClients = new Map<string, RclServiceClient>();

async function loadRclnodejs(): Promise<RclNodeJsModule> {
  try {
    // Dynamic import: rclnodejs is an *optional* dependency (see package.json)
    // that can only build/install inside a sourced ROS2 environment. This
    // import must never happen at module load time -- only when a mission-RPC
    // endpoint is actually called -- so the orchestrator still starts and
    // serves every other endpoint fine on a machine without ROS2.
    const rclnodejs = (await import("rclnodejs")) as unknown as RclNodeJsModule;
    return rclnodejs;
  } catch (error) {
    throw new Error(
      "rclnodejs is not available. Mission RPC (start mission / failsafe) requires " +
        "running the orchestrator in a sourced ROS2 environment on the fleet's " +
        `ROS_DOMAIN_ID. Underlying error: ${(error as Error).message}`,
    );
  }
}

async function getNode(): Promise<RclNode> {
  if (cachedNode) return cachedNode;
  const rclnodejs = await loadRclnodejs();
  await rclnodejs.init();
  cachedNode = rclnodejs.createNode("pennair_orchestrator");
  cachedNode.spin();
  return cachedNode;
}

async function getClient(serviceName: string): Promise<RclServiceClient> {
  const existing = cachedClients.get(serviceName);
  if (existing) return existing;
  const node = await getNode();
  const client = node.createClient("std_srvs/srv/Trigger", serviceName);
  cachedClients.set(serviceName, client);
  return client;
}

/** Real Trigger-service caller, backed by rclnodejs -- the default, unless a test injects a fake one. */
export const rclnodejsTriggerServiceCaller: TriggerServiceCaller = async (serviceName) => {
  const client = await getClient(serviceName);
  const available = await client.waitForService(5_000);
  if (!available) {
    return { success: false, message: `Service ${serviceName} is not available` };
  }
  const response = await client.sendRequest({});
  return { success: response.success, message: response.message };
};
