# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
from pathlib import Path

# -- Project information -----------------------------------------------------
project = "PennAiR Monorepo"
copyright = "Copyright © 2026, Penn Aerial Robotics"
author = "Penn Aerial Robotics"
# release = '0.1.0' # not gonna track version rn

exclude_patterns = ["build", "_build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output --------------------------------------------------
html_theme = "shibuya"
html_title = "PennAiR Monorepo Documentation"
html_css_files = ["docs-api.css"]
html_static_path = ["_static", "build/browser-assets"]  # path for extra static html elements, like favicons
html_logo = "_static/penn-air-full-logo.png"
html_favicon = "_static/pennair-preview-150x150.png"
html_theme_options = {
    "globaltoc_expand_depth": 1,  # Auto-expand the first level of every toctree branch on every page,
    "nav_socials": [
        {
            "name": "GitHub",
            "url": "https://github.com/pennaerial/monorepo",
            "icon": "simple-icons:github",
        }
    ],
    "accent_color": "indigo",
}

# if 1, skips autoapi build for faster local iteration on handwritten docs
SKIP_AUTOAPI_BUILD = os.environ.get("SKIP_AUTOAPI_BUILD") == "1"

REPO_ROOT = Path(__file__).resolve().parent.parent
ROS_WS_PATH = os.environ.get("PENNAIR_ROS_WS_PATH", str(REPO_ROOT / "ros_ws"))


def setup(app):
    app.add_config_value("skip_autoapi_build", SKIP_AUTOAPI_BUILD, "env")


sys.path.insert(0, str(Path(__file__).parent / "_ext"))  # make _ext modules discoverable

# -- General configuration ----------------------------------------------------
extensions = [
    "sphinx.ext.napoleon",  # for parsing Google and NumPy style docstrings
    "myst_parser",  # allows markdown doc writing
    "breathe",  # creates sphinx directives from doxygen xml
    "sphinx_design",
    "sphinx_new_tab_link",
    "sphinx_copybutton",
    "sphinx_tabs.tabs",
    "sphinx_iconify",
    # "sphinx_docsearch",
    # CUSTOM EXTENSIONS
    "pennair_contributors",
    "docs_status",
    "sphinxcontrib.mermaid",
    "sphinx.ext.mathjax",
]
#
# docsearch_app_id = "<DOCSEARCH_APP_ID>"
# docsearch_api_key = "<DOCSEARCH_SEARCH_API_KEY>"
# docsearch_index_name = "<DOCSEARCH_INDEX_NAME>"
#
if not SKIP_AUTOAPI_BUILD:
    extensions.append("autoapi.extension")
add_module_names = False  # object names don't show full header path so its not too verbose. E.g., uav.modes.LandingMode vs LandingMode

# =============================================================================
#                       SPHINX AUTOAPI CONFIGURATION
# =============================================================================
if not SKIP_AUTOAPI_BUILD:
    autoapi_dirs = [  # python only
        f"{ROS_WS_PATH}/src/common/uav/uav",
        f"{ROS_WS_PATH}/src/sae/payload/payload",
        f"{ROS_WS_PATH}/src/common/sim/sim",
        f"{ROS_WS_PATH}/src/common/vehicle_common/vehicle_common",
        f"{ROS_WS_PATH}/src/iarc/iarc_flight/iarc_flight",
    ]
    # Custom templates dir (only overrides python/module.rst). Used to customize names (favoring displaying short names over full module path names)
    autoapi_template_dir = "_templates/autoapi"
    autoapi_options = [
        "members",
        "undoc-members",
        "private-members",
        "show-inheritance",
        "show-module-summary",
        "special-members",
        # "imported-members", # comment out to remove duplicated re-exported classes at module level
    ]


# =============================================================================
#                               BREATHE CONFIGURATION
# =============================================================================

breathe_projects = {
    "payload_controller": str(Path(os.environ.get("PENNAIR_PAYLOAD_CONTROLLER_PATH", REPO_ROOT / "payload_controller")) / "doxygen/xml")
}

# Mermaid diagrams share a restrained palette and adapt to the site's color mode.
mermaid_version = "11.17.2"
mermaid_use_local = "mermaid/mermaid.esm.min.mjs"
d3_use_local = "d3/d3.min.js"
mermaid_init_config = {
    "startOnLoad": False,
    "flowchart": {"curve": "basis", "htmlLabels": False, "nodeSpacing": 28, "rankSpacing": 42},
    "themeVariables": {"fontFamily": "system-ui, sans-serif", "fontSize": "15px"},
}
mermaid_d3_zoom = True
mermaid_fullscreen = True
