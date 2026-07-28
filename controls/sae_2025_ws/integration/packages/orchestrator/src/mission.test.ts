import { describe, expect, it } from "vitest";
import type { TriggerServiceCaller } from "./ros/trigger-service-client.js";
import { startMission, triggerFailsafe } from "./mission.js";

describe("startMission", () => {
  it("calls the vehicle's mode_manager/start_mission service", async () => {
    const caller: TriggerServiceCaller = async (serviceName) => {
      expect(serviceName).toBe("/air-01/mode_manager/start_mission");
      return { success: true, message: "Mission runtime running" };
    };

    expect(await startMission("air-01", caller)).toEqual({
      success: true,
      output: "Mission runtime running",
    });
  });

  it("reports failure when the service call itself throws", async () => {
    const caller: TriggerServiceCaller = async () => {
      throw new Error("Service /air-01/mode_manager/start_mission is not available");
    };

    const result = await startMission("air-01", caller);
    expect(result.success).toBe(false);
    expect(result.error).toContain("not available");
  });

  it("reports failure when the service responds success:false", async () => {
    const caller: TriggerServiceCaller = async () => ({ success: false, message: "already running" });
    expect(await startMission("air-01", caller)).toEqual({ success: false, error: "already running" });
  });
});

describe("triggerFailsafe", () => {
  it("calls the vehicle's mode_manager/failsafe service", async () => {
    const caller: TriggerServiceCaller = async (serviceName) => {
      expect(serviceName).toBe("/air-01/mode_manager/failsafe");
      return { success: true, message: "failsafe triggered" };
    };

    expect(await triggerFailsafe("air-01", caller)).toEqual({
      success: true,
      output: "failsafe triggered",
    });
  });
});
