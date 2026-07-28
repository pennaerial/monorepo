import js from "@eslint/js";
import tseslint from "typescript-eslint";

export default tseslint.config(
  js.configs.recommended,
  ...tseslint.configs.recommended,
  {
    ignores: ["**/dist/**", "**/node_modules/**", "**/.expo/**"],
  },
  {
    // TypeScript's own compiler already catches undefined identifiers;
    // no-undef otherwise false-positives on ambient/DOM/Node globals.
    rules: {
      "no-undef": "off",
    },
  },
);
