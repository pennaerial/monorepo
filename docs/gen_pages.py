from pathlib import Path
import mkdocs_gen_files

SRC_ROOT = Path("controls/sae_2025_ws/src")

INCLUDED_PACKAGES = {
    "integration",
    "uav",
    "sim",
    "tools",
}

SKIP_FILES = {
    "__init__.py",
    "setup.py",
}

# Map of virtual doc path -> source README path (relative to repo root)
README_PAGES = {
    "index.md": "README.md",
    "controls/index.md": "controls/sae_2025_ws/README.md",
    "controls/uav.md": "controls/sae_2025_ws/src/uav/README.md",
    "controls/payload.md": "controls/sae_2025_ws/src/payload/README.md",
    "controls/integration.md": "controls/sae_2025_ws/src/integration/README.md",
    "controls/sim.md": "controls/sae_2025_ws/src/sim/README.md",
    "controls/tools.md": "controls/sae_2025_ws/src/tools/README.md",
    "controls/custom_sim_bridge.md": "controls/sae_2025_ws/src/custom_sim_bridge/README.md",
    "controls/custom_gz_plugins.md": "controls/sae_2025_ws/src/custom_gz_plugins/README.md",
    "sim/gazebo_harmonic.md": "sim/sae aero/gazebo harmonic/README.md",
}

for doc_path, readme in README_PAGES.items():
    src = Path(readme)
    if src.exists():
        with mkdocs_gen_files.open(doc_path, "w") as fd:
            fd.write(src.read_text())
        mkdocs_gen_files.set_edit_path(doc_path, src)

for package in INCLUDED_PACKAGES:
    for path in sorted((SRC_ROOT / package).rglob("*.py")):
        if path.name in SKIP_FILES:
            continue

        if not (path.parent / "__init__.py").exists():
            continue

        module_name = ".".join(
            path.relative_to(SRC_ROOT).with_suffix("").parts
        )

        doc_path = Path("API", *module_name.split(".")).with_suffix(".md")

        with mkdocs_gen_files.open(doc_path, "w") as fd:
            print(f"::: {module_name}", file=fd)

        mkdocs_gen_files.set_edit_path(doc_path, path)
