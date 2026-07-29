import { describe, expect, it } from "vitest";
import {
  inferMissionTarget,
  parseMissionDocument,
  parseTransitionEntries,
  publicModeId,
  renderMissionDocument,
  uniqueModeName,
  updateMissionDocumentMode,
} from "./mission-document.js";

describe("publicModeId", () => {
  it("strips the uav.modes. prefix", () => {
    expect(publicModeId("uav.modes.takeoff.Takeoff")).toBe("takeoff.Takeoff");
  });
  it("collapses a repeated trailing segment", () => {
    expect(publicModeId("foo.Bar.Bar")).toBe("foo.Bar");
  });
  it("returns '' for non-strings", () => {
    expect(publicModeId(undefined)).toBe("");
    expect(publicModeId(42)).toBe("");
  });
});

describe("inferMissionTarget", () => {
  it("infers payload from a payload. prefix or segment", () => {
    expect(inferMissionTarget("payload.corner_navigate.CornerNavigate")).toBe("payload");
    expect(inferMissionTarget("some.payload.thing")).toBe("payload");
  });
  it("infers uav from a uav. prefix or segment", () => {
    expect(inferMissionTarget("uav.takeoff.Takeoff")).toBe("uav");
  });
  it("returns '' when neither prefix matches", () => {
    expect(inferMissionTarget("other.Thing")).toBe("");
  });
});

describe("parseTransitionEntries", () => {
  it("parses key: value lines, skipping comments and blanks", () => {
    const raw = "success: cruise\n# a comment\n\nfailure: terminate\n";
    expect(parseTransitionEntries(raw)).toEqual([
      { key: "success", value: "cruise" },
      { key: "failure", value: "terminate" },
    ]);
  });
});

const SAMPLE_MISSION_YAML = `start_mode: takeoff
modes:
  takeoff:
    mode: uav.modes.takeoff.Takeoff
    params:
      target_altitude: 10.0
    transitions:
      success: cruise
      failure: terminate

  cruise:
    mode: uav.modes.cruise.Cruise
    params: {}
    transitions:
      success: land
`;

describe("parseMissionDocument / renderMissionDocument", () => {
  it("parses modes, params, and transitions from real mission YAML", () => {
    const doc = parseMissionDocument(SAMPLE_MISSION_YAML);
    expect(doc.warnings).toEqual([]);
    expect(doc.modes).toHaveLength(2);

    const [takeoff, cruise] = doc.modes;
    expect(takeoff.name).toBe("takeoff");
    expect(takeoff.mode).toBe("takeoff.Takeoff");
    expect(takeoff.paramsRaw).toBe("target_altitude: 10.0");
    expect(takeoff.transitions).toEqual([
      { key: "success", value: "cruise" },
      { key: "failure", value: "terminate" },
    ]);
    // publicModeId strips the "uav.modes." prefix before inferMissionTarget
    // ever sees it, so a mode ref under that exact convention loses its
    // target-inference signal -- this is the real (faithfully-ported)
    // behavior, not a gap introduced by the port. Modes that don't use the
    // "uav.modes." prefix specifically still infer correctly (see below).
    expect(takeoff.target).toBe("");
    expect(takeoff.hasParams).toBe(true);
    expect(takeoff.hasTransitions).toBe(true);

    expect(cruise.name).toBe("cruise");
    expect(cruise.transitions).toEqual([{ key: "success", value: "land" }]);
    expect(doc.selectedTarget).toBe("");
  });

  it("infers a target when the mode ref doesn't use the uav.modes. prefix convention", () => {
    const doc = parseMissionDocument(`modes:
  land:
    mode: uav.land.Land
    params: {}
    transitions: {}
`);
    expect(doc.modes[0].target).toBe("uav");
    expect(doc.selectedTarget).toBe("uav");
  });

  it("warns when there's no top-level modes mapping", () => {
    const doc = parseMissionDocument("start_mode: takeoff\n");
    expect(doc.modes).toEqual([]);
    expect(doc.warnings).toEqual(["Mission YAML does not contain a top-level modes mapping."]);
  });

  it("warns when modes is present but empty", () => {
    const doc = parseMissionDocument("modes:\n");
    expect(doc.warnings).toEqual(["Mission YAML has a modes block but no mode entries."]);
  });

  it("warns when modes mix targets", () => {
    const mixed = `modes:
  a:
    mode: uav.takeoff.Takeoff
    params: {}
    transitions: {}
  b:
    mode: payload.corner.Corner
    params: {}
    transitions: {}
`;
    const doc = parseMissionDocument(mixed);
    expect(doc.selectedTarget).toBe("");
    expect(doc.warnings[0]).toMatch(/mix targets/);
  });

  it("round-trips through render without losing modes/params/transitions", () => {
    const doc = parseMissionDocument(SAMPLE_MISSION_YAML);
    const rendered = renderMissionDocument(doc);
    const reparsed = parseMissionDocument(rendered);
    expect(reparsed.modes.map((m) => ({ name: m.name, mode: m.mode, transitions: m.transitions }))).toEqual(
      doc.modes.map((m) => ({ name: m.name, mode: m.mode, transitions: m.transitions })),
    );
  });

  it("falls back to rawText when there are no modes to render", () => {
    const doc = parseMissionDocument("start_mode: takeoff\n");
    expect(renderMissionDocument(doc)).toBe("start_mode: takeoff\n");
  });

  it("does not inject params: {} for a mode that never had a params key, mirroring hasTransitions", () => {
    const doc = parseMissionDocument(`modes:
  land:
    mode: uav.LandingMode
`);
    expect(doc.modes[0].hasParams).toBe(false);
    const rendered = renderMissionDocument(updateMissionDocumentMode(doc, "land", (mode) => ({ ...mode })));
    expect(rendered).not.toContain("params");
  });

  it("still emits a params representation for a mode that explicitly had an empty params key", () => {
    const doc = parseMissionDocument(`modes:
  land:
    mode: uav.LandingMode
    params: {}
`);
    expect(doc.modes[0].hasParams).toBe(true);
    // paramsRaw captures the literal "{}" text, which renders through the
    // nested-block path (valid, equivalent YAML, just different formatting
    // from the compact inline form) -- what matters here is hasParams:true
    // still causes *some* params key to survive, unlike the hasParams:false
    // case above where none should appear at all.
    const rendered = renderMissionDocument(updateMissionDocumentMode(doc, "land", (mode) => ({ ...mode })));
    expect(rendered).toMatch(/params:/);
  });
});

describe("updateMissionDocumentMode", () => {
  it("applies an updater to the named mode and resyncs rawText", () => {
    // mode.mode always holds the already-public-id form (post-publicModeId)
    // when read from parseMissionDocument -- updaters write that same form,
    // renderMissionDocument doesn't re-normalize on write.
    const doc = parseMissionDocument(SAMPLE_MISSION_YAML);
    const updated = updateMissionDocumentMode(doc, "takeoff", (mode) => ({ ...mode, mode: "takeoff.Takeoff2" }));
    const takeoff = updated.modes.find((m) => m.name === "takeoff");
    expect(takeoff?.mode).toBe("takeoff.Takeoff2");
    expect(updated.rawText).toContain("mode: takeoff.Takeoff2");
  });

  it("leaves other modes untouched", () => {
    const doc = parseMissionDocument(SAMPLE_MISSION_YAML);
    const updated = updateMissionDocumentMode(doc, "takeoff", (mode) => ({ ...mode, mode: "changed" }));
    const cruise = updated.modes.find((m) => m.name === "cruise");
    expect(cruise?.mode).toBe("cruise.Cruise");
  });
});

describe("uniqueModeName", () => {
  it("returns the base name if unused", () => {
    const doc = parseMissionDocument(SAMPLE_MISSION_YAML);
    expect(uniqueModeName(doc, "land")).toBe("land");
  });
  it("appends an incrementing suffix on collision", () => {
    const doc = parseMissionDocument(SAMPLE_MISSION_YAML);
    expect(uniqueModeName(doc, "takeoff")).toBe("takeoff_1");
  });
});
