import { describe, expect, it } from "vitest";
import { normalizeIntegrationFleetDocument } from "./fleet-document.js";

/**
 * Regression test for a surprising silent-data-loss path in
 * normalizeIntegrationFleetDocument (fleet-document.ts).
 *
 * `missionTargetByName[missionName]` used to be checked *before* the
 * vehicle's own already-explicit `kind`, not just as a fallback for when
 * `kind` is unset. So if a vehicle already had an explicit `kind: "uav"`
 * (chosen directly via the "kind" field in FleetEditor.tsx's vehicle.common
 * section, a real editable field per tools/schema-export/schema_export.py's
 * section_fields(("name", "mission", "mission_path", "kind"))), and its
 * `mission` field was set to a mission whose inferred target disagreed, the
 * mission's inferred target silently won over the user's explicit choice on
 * the next save/load -- and since normalizeIntegrationFleetDocument also
 * deletes px4_airframe_id/px4_namespace for any non-"uav" kind,
 * previously-set hardware fields for that vehicle were silently wiped too,
 * with no warning surfaced anywhere (FleetEditor.tsx's save() just calls
 * normalizeIntegrationFleetDocument and writes the result).
 *
 * This exact precedence was a faithful 1:1 port of the old dashboard's
 * App.jsx normalizeIntegrationFleetDocument (see git show
 * 37c30468^:controls/sae_2025_ws/src/integration/frontend/src/App.jsx around
 * line 814) -- not a new regression introduced by the port, but a real,
 * reachable bug worth fixing now that it's been found: an explicit user
 * choice should win over inference, not the other way around.
 */
describe("normalizeIntegrationFleetDocument respects an explicit kind over mission-target inference (regression)", () => {
  it("keeps a vehicle's explicitly-set kind even when the mission's inferred target disagrees, and does not drop px4 fields", () => {
    const doc = normalizeIntegrationFleetDocument(
      {
        vehicles: [
          {
            name: "air-01",
            kind: "uav", // explicitly chosen by the user via the FleetEditor kind field
            mission: "corner_pickup", // a mission whose registry target is "payload"
            px4_airframe_id: 5,
            px4_namespace: "air01",
          },
        ],
      },
      { corner_pickup: "payload" },
    );

    expect(doc.vehicles[0].kind).toBe("uav");
    expect(doc.vehicles[0]).toHaveProperty("px4_airframe_id", 5);
    expect(doc.vehicles[0]).toHaveProperty("px4_namespace", "air01");
  });

  it("still infers kind from the mission when the vehicle has no explicit kind set", () => {
    const doc = normalizeIntegrationFleetDocument(
      { vehicles: [{ name: "pay-01", mission: "corner_pickup" }] },
      { corner_pickup: "payload" },
    );
    expect(doc.vehicles[0].kind).toBe("payload");
  });
});
