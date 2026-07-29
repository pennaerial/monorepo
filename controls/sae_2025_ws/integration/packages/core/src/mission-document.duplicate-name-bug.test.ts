import { describe, expect, it } from "vitest";
import { parseMissionDocument, updateMissionDocumentMode } from "./mission-document.js";

/**
 * Documents why MissionEditor.tsx's mode rename handler must validate
 * uniqueness (and reject the reserved "start" name) *before* ever calling
 * `updateMissionDocumentMode` -- fixed there (`renameSelectedMode`), not
 * here. `updateMissionDocumentMode`'s `doc.modes.map((mode) => mode.name ===
 * modeName ? updater(...) : mode)` matches *every* mode with a given name by
 * design (it's the same mechanism `addMode`/every field edit relies on), so
 * if the UI ever let two modes collide, this pure function would apply
 * edits to both at once and `removeMode`'s filter would remove both
 * simultaneously -- silently corrupting the mode the user can no longer even
 * see/select. Both assertions below pass: they document the core function's
 * real (by-design, name-keyed) contract, which is exactly why the guard has
 * to live in the component, not here.
 */
describe("mission mode rename collisions (regression)", () => {
  it("allows renaming a mode to collide with an existing mode's name (including the reserved 'start')", () => {
    const doc = parseMissionDocument(
      "modes:\n  start:\n    mode: uav.vtol.TakeoffMode\n    params: {}\n  cruise:\n    mode: uav.NavGPSMode\n    params: {}\n",
    );
    expect(doc.modes.map((m) => m.name)).toEqual(["start", "cruise"]);

    // Simulates the Name-field onChange handler: rename "cruise" -> "start".
    const renamed = updateMissionDocumentMode(doc, "cruise", (mode) => ({ ...mode, name: "start" }));

    // Bug: nothing prevented this. Two modes now share the reserved name.
    expect(renamed.modes.map((m) => m.name)).toEqual(["start", "start"]);
  });

  it("applies a subsequent edit to BOTH colliding modes at once, corrupting the one the UI can no longer address", () => {
    const doc = parseMissionDocument(
      "modes:\n  start:\n    mode: uav.vtol.TakeoffMode\n    params: {}\n  cruise:\n    mode: uav.NavGPSMode\n    params: {}\n",
    );
    const renamed = updateMissionDocumentMode(doc, "cruise", (mode) => ({ ...mode, name: "start" }));

    // MissionEditor's `selectedMode = doc.modes.find(m => m.name === "start")`
    // would only ever show the FIRST "start" entry to the user...
    const visibleToUser = renamed.modes.find((m) => m.name === "start");
    expect(visibleToUser?.mode).toBe("uav.vtol.TakeoffMode");

    // ...but any further edit keyed by that same name silently mutates BOTH.
    const edited = updateMissionDocumentMode(renamed, "start", (mode) => ({ ...mode, mode: "uav.LandingMode" }));
    const stillNamedStart = edited.modes.filter((m) => m.name === "start");
    expect(stillNamedStart).toHaveLength(2);
    // Bug: the user thought they edited one mode; they edited two,
    // including the original real entrypoint mode's target class, silently.
    expect(stillNamedStart.map((m) => m.mode)).toEqual(["uav.LandingMode", "uav.LandingMode"]);
  });
});
