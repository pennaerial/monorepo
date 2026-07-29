import { describe, expect, it } from "vitest";
import * as YAML from "yaml";
import { getYamlBlockValue, setYamlBlockValue } from "./yaml-text.js";

/**
 * Regression test for a real data-corruption bug found while testing Phase
 * 4's mission-graph editor against real mission YAML from the repo (e.g.
 * controls/sae_2025_ws/src/uav/missions/mission.yaml's `GPS` mode):
 *
 *   coordinates:
 *     - [[5, 10, -10], 1, LOCAL]
 *
 * `getYamlBlockValue` correctly extracts this single-item YAML block
 * sequence as the raw string "- [[5, 10, -10], 1, LOCAL]" (including the
 * leading "- " block-sequence indicator, since it's not a real parser).
 *
 * `setYamlBlockValue` used to only nest a value as `key:\n  <indented
 * value>` when the value contained an embedded "\n" -- a single-item block
 * sequence's value has no embedded newline, so it took the inline branch and
 * got written back as `coordinates: - [[5, 10, -10], 1, LOCAL]` on one line,
 * which is not valid YAML (a block-sequence "-" can't follow "key:" on the
 * same line). Fixed by also checking whether the trimmed value itself starts
 * with a block-sequence indicator.
 */
describe("setYamlBlockValue with a single-item YAML block sequence (regression)", () => {
  it("round-trips a real-shaped `coordinates:\\n  - [[...]]` block value back into valid YAML", () => {
    const original = "coordinates:\n  - [[5, 10, -10], 1, LOCAL]";
    const value = getYamlBlockValue(original, "coordinates");
    expect(value).toBe("- [[5, 10, -10], 1, LOCAL]");

    const rewritten = setYamlBlockValue(original, "coordinates", value);

    expect(rewritten).toBe(original);
    expect(() => YAML.parse(rewritten)).not.toThrow();
  });
});
