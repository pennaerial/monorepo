import { describe, expect, it } from "vitest";
import {
  dedentBlock,
  escapeRegex,
  getYamlBlockValue,
  getYamlFieldValue,
  indentBlock,
  parseInlineYamlValue,
  parseYamlScalar,
  serializeYamlScalar,
  setYamlBlockValue,
  setYamlFieldValue,
  splitInlineComment,
  splitTopLevelYamlEntries,
  stripYamlComment,
  unquoteYamlScalar,
} from "./yaml-text.js";

describe("escapeRegex", () => {
  it("escapes regex metacharacters", () => {
    expect(escapeRegex("a.b*c")).toBe("a\\.b\\*c");
  });
});

describe("splitInlineComment", () => {
  it("splits at the first ' #'", () => {
    expect(splitInlineComment("value # a comment")).toEqual({ value: "value", comment: " # a comment" });
  });
  it("returns no comment when none present", () => {
    expect(splitInlineComment("plain value")).toEqual({ value: "plain value", comment: "" });
  });
});

describe("unquoteYamlScalar", () => {
  it("strips matching single or double quotes", () => {
    expect(unquoteYamlScalar("'hi'")).toBe("hi");
    expect(unquoteYamlScalar('"hi"')).toBe("hi");
    expect(unquoteYamlScalar("hi")).toBe("hi");
  });
});

describe("parseYamlScalar / serializeYamlScalar", () => {
  it("round-trips booleans", () => {
    expect(parseYamlScalar("true", "boolean")).toBe(true);
    expect(parseYamlScalar("FALSE", "boolean")).toBe(false);
    expect(serializeYamlScalar(true, "boolean")).toBe("true");
  });
  it("round-trips integers, defaulting to 0 on garbage", () => {
    expect(parseYamlScalar("42", "integer")).toBe(42);
    expect(parseYamlScalar("nope", "integer")).toBe(0);
    expect(serializeYamlScalar(7, "integer")).toBe("7");
  });
  it("round-trips array3, defaulting to [0,0,0] on wrong length", () => {
    expect(parseYamlScalar("[1, 2, 3]", "array3")).toEqual([1, 2, 3]);
    expect(parseYamlScalar("[1, 2]", "array3")).toEqual([0, 0, 0]);
    expect(serializeYamlScalar([1, 2, 3], "array3")).toBe("[1, 2, 3]");
  });
  it("quotes strings containing colon/hash/whitespace, passes plain strings through", () => {
    expect(serializeYamlScalar("plain", "text")).toBe("plain");
    expect(serializeYamlScalar("has: colon", "text")).toBe("'has: colon'");
    expect(serializeYamlScalar("", "text")).toBe("''");
  });
});

describe("getYamlFieldValue / setYamlFieldValue", () => {
  it("gets a flat key's value anywhere in the content", () => {
    const content = "foo: 1\nbar: hello\n";
    expect(getYamlFieldValue(content, "bar", "text")).toBe("hello");
  });
  it("returns type-appropriate defaults when the key is absent", () => {
    expect(getYamlFieldValue("", "missing", "boolean")).toBe(false);
    expect(getYamlFieldValue("", "missing", "array3")).toEqual([0, 0, 0]);
    expect(getYamlFieldValue("", "missing", "text")).toBe("");
  });
  it("sets an existing key in place, preserving inline comments", () => {
    const content = "debug: false # keep this\nmission: foo\n";
    const updated = setYamlFieldValue(content, "debug", "boolean", true);
    expect(updated).toContain("debug: true # keep this");
  });
  it("appends the key when absent", () => {
    const updated = setYamlFieldValue("mission: foo", "debug", "boolean", true);
    expect(updated).toContain("debug: true");
  });
});

describe("indentBlock / dedentBlock", () => {
  it("indents non-blank lines only", () => {
    expect(indentBlock("a\n\nb", 2)).toBe("  a\n\n  b");
  });
  it("dedents up to the given width and trims", () => {
    expect(dedentBlock("      a\n      b\n", 6)).toBe("a\nb");
  });
});

describe("parseInlineYamlValue / stripYamlComment", () => {
  it("extracts the value after the first colon", () => {
    expect(parseInlineYamlValue("key: value")).toBe("value");
    expect(parseInlineYamlValue("no colon")).toBe("");
  });
  it("strips a trailing ' #' comment", () => {
    expect(stripYamlComment("value # comment")).toBe("value");
    expect(stripYamlComment("value")).toBe("value");
  });
});

describe("splitTopLevelYamlEntries / getYamlBlockValue / setYamlBlockValue", () => {
  const fragment = "target_position: [1, 2, 3]\napproach_speed: 0.5\nnested:\n  a: 1\n  b: 2\n";

  it("splits top-level entries with their line spans", () => {
    const entries = splitTopLevelYamlEntries(fragment);
    expect(entries.map((e) => e.key)).toEqual(["target_position", "approach_speed", "nested"]);
  });

  it("gets an inline scalar value", () => {
    expect(getYamlBlockValue(fragment, "approach_speed")).toBe("0.5");
  });

  it("gets a nested block value, dedented", () => {
    expect(getYamlBlockValue(fragment, "nested")).toBe("a: 1\nb: 2");
  });

  it("returns '' for an absent key", () => {
    expect(getYamlBlockValue(fragment, "missing")).toBe("");
  });

  it("sets a new key when absent", () => {
    const updated = setYamlBlockValue("", "speed", "1.5");
    expect(updated).toBe("speed: 1.5");
  });

  it("replaces an existing key's value", () => {
    const updated = setYamlBlockValue("speed: 1.5\nother: x", "speed", "2.0");
    expect(updated).toContain("speed: 2.0");
    expect(updated).toContain("other: x");
  });

  it("removes the key entirely when set to an empty value", () => {
    const updated = setYamlBlockValue("speed: 1.5\nother: x", "speed", "");
    expect(updated).not.toContain("speed");
    expect(updated).toContain("other: x");
  });

  it("writes a multi-line value as an indented nested block", () => {
    const updated = setYamlBlockValue("", "nested", "a: 1\nb: 2");
    expect(updated).toBe("nested:\n  a: 1\n  b: 2");
  });
});
