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
html_static_path = ["_static"]  # path for extra static html elements, like favicons
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
SKIP_AUTOAPI_BUILD = (os.environ.get("SKIP_AUTOAPI_BUILD") == "1")

SAE_WS_PATH = os.environ["PENNAIR_SAE_WS_PATH"]

sys.path.insert(0, str(Path(__file__).parent / "_ext")) # make _ext modules discoverable

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

    # CUSTOM EXTENSIONS
    "pennair_contributors",
]
if not SKIP_AUTOAPI_BUILD:
    extensions.append("autoapi.extension")
add_module_names = False  # object names don't show full header path so its not too verbose. E.g., uav.modes.LandingMode vs LandingMode

# =============================================================================
#                       SPHINX AUTOAPI CONFIGURATION
# =============================================================================
if not SKIP_AUTOAPI_BUILD:
    autoapi_dirs = [  # python only
        f"{SAE_WS_PATH}/src/uav/uav",
        f"{SAE_WS_PATH}/src/payload/payload",
        f"{SAE_WS_PATH}/src/sim/sim",
        f"{SAE_WS_PATH}/src/vehicle_common/vehicle_common",
    ]
    # Custom templates dir (only overrides python/module.rst). Used to customize names (favoring displaying short names over full module path names)
    autoapi_template_dir = "../sphinx/_templates/autoapi"
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
    "payload_controller": f"{os.environ['PENNAIR_PAYLOAD_CONTROLLER_PATH']}/docs/doxygen/xml"
}
