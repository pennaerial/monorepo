"""Generate one documentation page per ROS package."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mkdocs_gen_files
from doc_utils import find_ros_packages, REPO_ROOT

src = REPO_ROOT / "controls" / "sae_2025_ws" / "src"


def _first_paragraph(readme: Path) -> str:
    """Return the first non-heading paragraph from a README, or empty string."""
    lines = readme.read_text().splitlines()
    paragraph: list[str] = []
    in_paragraph = False
    for line in lines:
        if line.startswith("#"):
            if in_paragraph:
                break
            continue
        if line.strip() == "":
            if in_paragraph:
                break
            continue
        paragraph.append(line.strip())
        in_paragraph = True
    return " ".join(paragraph)


nav = mkdocs_gen_files.Nav()

# --- landing page ---
packages = sorted(find_ros_packages(src, ament_type="all"), key=lambda p: p.name)

with mkdocs_gen_files.open("packages/index.md", "w") as fd:
    fd.write("# Packages\n\n")
    fd.write("| Package | Description |\n")
    fd.write("|---|---|\n")
    for pkg_dir in packages:
        pkg_name = pkg_dir.name
        readme = pkg_dir / "README.md"
        desc = _first_paragraph(readme) if readme.exists() else ""
        fd.write(f"| [`{pkg_name}`]({pkg_name}/README.md) | {desc} |\n")

nav[()] = "index.md"

# --- per-package pages ---
for pkg_dir in packages:
    pkg_name = pkg_dir.name
    readme = pkg_dir / "README.md"
    docs_dir = pkg_dir / "docs"

    supplementary = sorted(
        f for f in docs_dir.iterdir() if f.is_file() and f.suffix == ".md"
    ) if docs_dir.is_dir() else []

    if readme.exists():
        dest = Path("packages", pkg_name, "README.md")
        with mkdocs_gen_files.open(dest, "w") as fd:
            fd.write(readme.read_text())
        mkdocs_gen_files.set_edit_path(dest, readme.relative_to(REPO_ROOT))
        nav[(pkg_name,)] = Path(pkg_name, "README.md").as_posix()
    elif supplementary:
        first = supplementary.pop(0)
        dest = Path("packages", pkg_name, "docs", first.name)
        with mkdocs_gen_files.open(dest, "w") as fd:
            fd.write(first.read_text())
        mkdocs_gen_files.set_edit_path(dest, first.relative_to(REPO_ROOT))
        nav[(pkg_name,)] = Path(pkg_name, "docs", first.name).as_posix()
    else:
        dest = Path("packages", pkg_name, "README.md")
        with mkdocs_gen_files.open(dest, "w") as fd:
            fd.write(f"# `{pkg_name}`\n\n*No documentation yet.*\n")
        nav[(pkg_name,)] = Path(pkg_name, "README.md").as_posix()

    for doc_file in supplementary:
        dest = Path("packages", pkg_name, "docs", doc_file.name)
        with mkdocs_gen_files.open(dest, "w") as fd:
            fd.write(doc_file.read_text())
        mkdocs_gen_files.set_edit_path(dest, doc_file.relative_to(REPO_ROOT))
        nav[(pkg_name, doc_file.stem)] = Path(pkg_name, "docs", doc_file.name).as_posix()

with mkdocs_gen_files.open("packages/SUMMARY.md", "w") as nav_file:
    nav_file.writelines(nav.build_literate_nav())
