# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
project = 'PennAiR Monorepo'
copyright = '2026, Penn Aerial Robotics'
author = 'Penn Aerial Robotics'
# release = '0.1.0' # not gonna track version rn

# -- General configuration ----------------------------------------------------
extensions = []

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output --------------------------------------------------
html_theme = 'alabaster'
html_static_path = ['assets'] # path for extra static html elements, like favicons
