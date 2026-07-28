export interface TriggerResult {
  success: boolean;
  message: string;
}

/** Calls a `std_srvs/srv/Trigger` ROS2 service and returns its response. */
export type TriggerServiceCaller = (serviceName: string) => Promise<TriggerResult>;

// Verified against the real published rclnodejs@2.1.1 type declarations
// (downloaded via `npm pack`, since the package can't actually build/install
// without a sourced ROS2 environment -- see loadRclnodejs below). Two things
// were wrong in an earlier draft, caught by review rather than by tests
// (nothing here can be exercised against the real package in this repo's
// dev/CI environment): `sendRequest` is callback-based and returns nothing --
// the promise API is `sendRequestAsync`. And `createClient`'s `typeClass`
// argument must be the class object returned by `rclnodejs.require(...)`,
// not a plain string.
interface RclNodeJsModule {
  init(): Promise<void>;
  createNode(name: string): RclNode;
  shutdown(): void;
  require(interfaceName: string): unknown;
}

interface RclNode {
  createClient(typeClass: unknown, name: string): RclServiceClient;
  spin(): void;
}

interface RclServiceClient {
  waitForService(timeoutMs: number): Promise<boolean>;
  sendRequestAsync(
    request: Record<string, never>,
    options?: { timeout?: number },
  ): Promise<{ success: boolean; message: string }>;
}

async function loadRclnodejs(): Promise<RclNodeJsModule> {
  try {
    // Dynamic import: rclnodejs is an *optional* dependency (see package.json)
    // that can only build/install inside a sourced ROS2 environment. This
    // import must never happen at module load time -- only when a mission-RPC
    // endpoint is actually called -- so the orchestrator still starts and
    // serves every other endpoint fine on a machine without ROS2. Node's ESM
    // loader caches dynamic imports, so calling this repeatedly is cheap.
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

async function initNode(): Promise<RclNode> {
  const rclnodejs = await loadRclnodejs();
  await rclnodejs.init();
  const node = rclnodejs.createNode("pennair_orchestrator");
  node.spin();
  return node;
}

async function loadTriggerType(): Promise<unknown> {
  const rclnodejs = await loadRclnodejs();
  return rclnodejs.require("std_srvs/srv/Trigger");
}

// Cache the in-flight *promise*, not just the eventual value. This gives two
// things a plain "cache the resolved value" approach doesn't:
//  - single-flight: concurrent first calls (e.g. two simultaneous start-mission
//    requests before anything is initialized) all await the same promise
//    instead of racing to independently call rclnodejs.init()/createNode().
//  - failure doesn't poison the cache forever: if initialization throws at any
//    point, the cached promise is reset to null so the *next* call gets a
//    fresh attempt, rather than permanently caching a broken result with no
//    recovery short of a restart.
let nodePromise: Promise<RclNode> | null = null;
let triggerTypePromise: Promise<unknown> | null = null;

function getNode(): Promise<RclNode> {
  if (!nodePromise) {
    nodePromise = initNode().catch((error: unknown) => {
      nodePromise = null;
      throw error;
    });
  }
  return nodePromise;
}

function getTriggerType(): Promise<unknown> {
  if (!triggerTypePromise) {
    triggerTypePromise = loadTriggerType().catch((error: unknown) => {
      triggerTypePromise = null;
      throw error;
    });
  }
  return triggerTypePromise;
}

const clientPromises = new Map<string, Promise<RclServiceClient>>();

function getClient(serviceName: string): Promise<RclServiceClient> {
  const existing = clientPromises.get(serviceName);
  if (existing) return existing;

  const promise = Promise.all([getNode(), getTriggerType()])
    .then(([node, triggerType]) => node.createClient(triggerType, serviceName))
    .catch((error: unknown) => {
      clientPromises.delete(serviceName);
      throw error;
    });
  clientPromises.set(serviceName, promise);
  return promise;
}

/** Real Trigger-service caller, backed by rclnodejs -- the default, unless a test injects a fake one. */
export const rclnodejsTriggerServiceCaller: TriggerServiceCaller = async (serviceName) => {
  const client = await getClient(serviceName);
  const available = await client.waitForService(5_000);
  if (!available) {
    return { success: false, message: `Service ${serviceName} is not available` };
  }
  const response = await client.sendRequestAsync({}, { timeout: 10_000 });
  return { success: response.success, message: response.message };
};
