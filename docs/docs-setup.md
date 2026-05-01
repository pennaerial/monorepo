# Documentation Setup

Docs are built with [MkDocs](https://www.mkdocs.org/) via the `properdocs` wrapper. The config is `properdocs.yml` at the repo root.

## Serving locally

```bash
properdocs serve
```

## Plugins

| Plugin | Role |
|---|---|
| `mkdocs-gen-files` | Runs Python scripts at build time to generate virtual doc pages |
| `mkdocstrings` | Renders Python API reference from docstrings |
| `mkdoxy` | Renders C++ API reference via Doxygen |
| `literate-nav` | Reads `SUMMARY.md` files to build nav trees for generated sections |
| `section-index` | Makes section headers in the nav clickable (links to their `index.md` / `README.md`) |

## Auto-generated sections

Two scripts in `docs/` run at build time via `mkdocs-gen-files`:

### `docs/gen_ref_pages.py` — Python API reference

Finds every ament_python ROS package under `controls/sae_2025_ws/src/`, then for each one finds direct child directories with an `__init__.py` (the importable module roots). Walks each module and generates a `reference/<module>/<file>.md` page with a `:::` mkdocstrings directive. Output nav is written to `reference/SUMMARY.md`.

### `docs/gen_pkg_pages.py` — Package docs

Finds every ROS package (all build types, submodules excluded). For each:

- Copies `README.md` from the package root → `packages/<name>/README.md` (section index)
- Copies `docs/*.md` → `packages/<name>/docs/*.md` (supplementary pages, preserving the `docs/` prefix so relative links in the README stay valid)
- Generates a stub if neither exists

Also generates `packages/index.md` as a landing page with a one-line description pulled from the first paragraph of each package's README. Output nav is written to `packages/SUMMARY.md`.

### `docs/doc_utils.py` — Shared helpers

`find_ros_packages(src, ament_type, exclude_submodules)` — detects ROS packages by finding `package.xml` files one level deep under `src/`. Git submodules are excluded by parsing `.gitmodules`. The `ament_type` parameter filters to `"python"`, `"cmake"`, or `"all"`.

## Adding docs to a package

1. Add a `README.md` at the package root for the overview (this becomes the section landing page).
2. Add supplementary pages to `<package>/docs/*.md` — link to them from the README as `docs/quickstart.md` etc.
3. No config changes needed — packages are auto-detected.

## Adding C++ API docs (Doxygen)

Add a new project under `mkdoxy.projects` in `properdocs.yml`:

```yaml
- mkdoxy:
    projects:
      my_package:
        src-dirs: controls/sae_2025_ws/src/my_package/include controls/sae_2025_ws/src/my_package/src
        full-doc: true
        doxy-cfg:
          OUTPUT_LANGUAGE: English
          EXTRACT_ALL: YES
```

Then add a nav entry under `API Reference > C++`.
