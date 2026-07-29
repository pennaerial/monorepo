import { describe, expect, it } from "vitest";
import { parseMissionDocument, renderMissionDocument, updateMissionDocumentMode } from "./mission-document.js";

/**
 * Regression test for a real data-loss bug: several real mission YAML files
 * in the repo have a header comment before the top-level `modes:` key, e.g.
 * controls/sae_2025_ws/src/payload/missions/payload_apriltag_approach.yaml:
 *
 *   # Payload approaches a single AprilTag and terminates cleanly once close enough.
 *   modes:
 *     start:
 *       ...
 *
 * (also: uav/missions/test_straight_course.yaml, payload/missions/
 * payload_corner_navigate_sim.yaml, payload_dlz_navigate_sim.yaml,
 * payload_dual_approach_sim.yaml.)
 *
 * `renderMissionDocument` used to rebuild the document from scratch as
 * `["modes:", ...doc.modes...]` whenever `doc.modes.length` was truthy,
 * discarding any content that appeared before the `modes:` line -- header
 * comments, or any other top-level key -- the moment a single field of a
 * single mode was edited and the document re-rendered (which is exactly
 * what `updateMissionDocumentMode`, used by every field edit in
 * MissionEditor.tsx, does on every call). Fixed by capturing that leading
 * content as `ParsedMissionDocument.preamble` at parse time and
 * re-prepending it on render.
 *
 * The existing round-trip test in mission-document.test.ts
 * ("round-trips through render without losing modes/params/transitions")
 * uses a SAMPLE_MISSION_YAML that itself starts with `start_mode: takeoff\n`
 * before `modes:`, but only asserts that modes/mode/transitions survive --
 * it never checked whether that leading line survives, so this regression
 * went uncaught until now.
 */
describe("renderMissionDocument preamble preservation (regression)", () => {
  it("keeps a header comment that appeared before `modes:` across a single-field edit", () => {
    const original = "# Payload approaches a single AprilTag and terminates cleanly once close enough.\nmodes:\n  start:\n    mode: payload.PayloadAprilTagApproachMode\n    params:\n      tag_id: 0\n";
    const doc = parseMissionDocument(original);
    expect(doc.modes).toHaveLength(1);

    // Editing an unrelated field (mode name stays the same, just re-synced).
    const updated = updateMissionDocumentMode(doc, "start", (mode) => ({ ...mode }));

    expect(updated.rawText).toContain("# Payload approaches a single AprilTag and terminates cleanly once close enough.");
  });

  it("keeps a leading top-level key (e.g. start_mode:) the same way", () => {
    const original = "start_mode: takeoff\nmodes:\n  takeoff:\n    mode: uav.modes.takeoff.Takeoff\n    params: {}\n";
    const doc = parseMissionDocument(original);
    const rendered = renderMissionDocument(updateMissionDocumentMode(doc, "takeoff", (mode) => ({ ...mode })));
    expect(rendered).toContain("start_mode: takeoff");
  });
});
