import { rclnodejsTriggerServiceCaller, type TriggerServiceCaller } from "./ros/trigger-service-client.js";

interface SimpleResult {
  success: boolean;
  output?: string;
  error?: string;
}

/**
 * Calls a vehicle's mode_manager Trigger service directly over the ROS2
 * graph -- the orchestrator is a real ROS2 participant (see
 * ros/trigger-service-client.ts), so this reaches any Pi's mode_manager
 * node with no new node or process required on that Pi.
 */
async function callModeManagerTrigger(
  vehicleName: string,
  serviceName: string,
  caller: TriggerServiceCaller,
): Promise<SimpleResult> {
  try {
    const result = await caller(`/${vehicleName}/mode_manager/${serviceName}`);
    if (!result.success) {
      return { success: false, error: result.message || `${serviceName} failed` };
    }
    return { success: true, output: result.message || `${serviceName} called` };
  } catch (error) {
    return { success: false, error: (error as Error).message };
  }
}

export async function startMission(
  vehicleName: string,
  caller: TriggerServiceCaller = rclnodejsTriggerServiceCaller,
): Promise<SimpleResult> {
  return callModeManagerTrigger(vehicleName, "start_mission", caller);
}

// UAV-specific -- payload targets don't define this service (mirrors the
// old dashboard's target-kind branching); calling it against a payload
// vehicle will just fail with "service not available", which is acceptable
// for this phase since fleet/vehicle-kind metadata isn't wired up yet.
export async function triggerFailsafe(
  vehicleName: string,
  caller: TriggerServiceCaller = rclnodejsTriggerServiceCaller,
): Promise<SimpleResult> {
  return callModeManagerTrigger(vehicleName, "failsafe", caller);
}
