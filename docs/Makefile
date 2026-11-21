# Minimal makefile for Sphinx documentation

# You can set these variables from the command line, and also
# from the environment, for the first two.
SPHINXOPTS      ?=
SPHINXBUILD     ?= sphinx-build
SPHINXAUTOBUILD ?= sphinx-autobuild
SOURCEDIR       = pages/
CONFDIR         = .
BUILDDIR        ?= build
SERVEPORT       ?= 8000

# Opt-in: keep AutoAPI's generated .rst files (under docs/pages/autoapi/) around
# after the build instead of deleting them, and skip regenerating them next
# time if the underlying source files haven't changed. Off by default since
# kept files are real files left sitting in docs/pages/. Enable with:
#   make html AUTOAPI_KEEP_FILES=1
AUTOAPI_KEEP_FILES ?=
ifeq ($(AUTOAPI_KEEP_FILES),1)
SPHINXOPTS += -D autoapi_keep_files=1
endif

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" -c "$(CONFDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile serve start browser-assets doxygen

start:
	./start.sh

browser-assets:
	python3 prepare-assets.py

doxygen:
	./generate-doxygen.sh

html: browser-assets doxygen
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" -c "$(CONFDIR)" $(SPHINXOPTS) $(O)

serve: browser-assets doxygen
	@$(SPHINXAUTOBUILD) "$(SOURCEDIR)" "$(BUILDDIR)/html" -c "$(CONFDIR)" \
		--port $(SERVEPORT) \
		--ignore "$(SOURCEDIR)/autoapi" \
		--watch "_static" --watch "_templates" --watch "_ext" --watch "conf.py" \
		$(SPHINXOPTS) $(O)

