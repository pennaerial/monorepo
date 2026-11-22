Editing and Previewing Docs
===========================

Edit a source page, save it, and check the result in your browser. This guide
covers local startup, writing pages, diagrams, and recording page status.
The site uses **Sphinx**, a tool that turns text files into HTML, and the
**Shibuya** theme for its layout and styling.

Running Docs Locally
--------------------

Tools needed once
~~~~~~~~~~~~~~~~~

Use Bash in the repository's Ubuntu development environment. You need:

* Python 3 with virtual-environment support. **uv** is recommended; the startup
  script uses it to create a Python 3.12 environment, matching docs CI. Without
  uv, the script uses ``python3 -m venv`` and pip with your installed Python.
* **Node.js 22 or newer and npm** for the local Mermaid/D3 browser libraries.
  Installing the Python requirements alone does not install those JavaScript libraries.
* **Make**, which runs the named commands below.
* **Doxygen**, which extracts the C++ API reference from firmware source comments.

On Ubuntu, install the system packages if they are missing:

.. code-block:: bash

   sudo apt update
   sudo apt install python3 python3-venv python3-pip make doxygen

Use the `official Node.js installation page <https://nodejs.org/en/download>`_
for Node.js 22 or newer with npm; an older system package may be too old.
For uv, follow its `installation guide
<https://docs.astral.sh/uv/getting-started/installation/>`_. Check your commands with:

.. code-block:: bash

   python3 --version
   node --version
   npm --version
   make --version
   doxygen --version

First startup downloads Python/npm dependencies, so it needs internet access.
You do not need to activate ROS or ESP-IDF to edit documentation.

Start the preview
~~~~~~~~~~~~~~~~~

Stop an older preview with ``Ctrl+C`` in its terminal. From the repository root:

.. code-block:: bash

   cd docs
   make start

Open `http://localhost:8000/ <http://localhost:8000/>`_ after the build finishes.
Keep the terminal running while editing; stop the preview with ``Ctrl+C``.
The command works from ``docs/`` every time, including after dependency changes.

``make start`` runs ``start.sh``, which:

1. Sources the repository's ``dev_env.sh`` to set project paths.
2. Creates ``docs/.venv`` and installs ``docs-requirements.txt`` when its saved
   requirements differ. This is separate from the older ``docs/venv`` directory.
3. Runs the normal Make preview target. It installs the exact npm dependency
   versions from ``package-lock.json`` when needed, then copies Mermaid/D3 into
   the static files served with the site.
4. Runs Doxygen, then starts ``sphinx-autobuild``, which rebuilds pages and asks
   the browser to refresh when watched files change. Startup uses ``-E -a`` to
   discard Sphinx's saved environment and rewrite pages.

Startup skips Python API generation by default to keep handwritten-doc previews
faster. It still includes the C++ reference. For the Python reference as well:

.. code-block:: bash

   SKIP_AUTOAPI_BUILD=0 make start

If port 8000 is occupied, stop the old preview or select another port:

.. code-block:: bash

   SERVEPORT=8001 make start

Open ``http://localhost:8001/`` for that command. If you deliberately run two
previews at once, give the second its own output folder as well:

.. code-block:: bash

   SERVEPORT=8001 BUILDDIR=build/preview-8001 make start

Changing only the port still leaves both processes writing the same generated
files and Sphinx cache. The usual single-preview output is ``docs/build/html/``;
the last example writes to ``docs/build/preview-8001/html/``.

Make a build without a server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

From ``docs/``, use the same managed environment and dependency preparation:

.. code-block:: bash

   ./start.sh html

Include the Python reference with:

.. code-block:: bash

   SKIP_AUTOAPI_BUILD=0 ./start.sh html

After initial setup, the lower-level commands are also available:

.. code-block:: bash

   source .venv/bin/activate
   SKIP_AUTOAPI_BUILD=1 make serve
   # Or build once:
   # SKIP_AUTOAPI_BUILD=1 make html

Unlike ``make start``, these direct commands expect Python dependencies to be
installed already and do not force a fresh Sphinx environment. Without
``SKIP_AUTOAPI_BUILD=1``, the configuration enables Python API generation.

Doxygen and internet access
~~~~~~~~~~~~~~~~~~~~~~~~~~~

When installed, Doxygen refreshes the C++ XML before preview/build startup.
Without it, the script can reuse an existing XML index only if the Doxyfile and
C/C++ source files are not newer than that index. Otherwise it stops and asks you
to install Doxygen. A fresh checkout needs Doxygen; generated XML is not committed.

The homepage normally requests GitHub contribution counts. To skip that request:

.. code-block:: bash

   PENNAIR_DOCS_OFFLINE=1 make start

Contributor cards still appear, sorted alphabetically. This setting skips that
GitHub request; it does not turn a first-time dependency install into an offline
install. After dependency setup, Mermaid and D3 are served locally. MathJax for
formulas and some icon assets may still need internet access in the browser.

Edit a page and see the result
------------------------------

For a first experiment, edit ``docs/pages/pc/overview.rst``, save a sentence,
and visit :doc:`pc/overview` in the local preview. Edit the source file, not
anything under ``docs/build/``: a later build overwrites generated HTML.

.. list-table:: What to do after a change
   :header-rows: 1
   :widths: 38 62

   * - Changed file
     - Preview action
   * - A page under ``docs/pages/``
     - Save and wait for rebuilding to finish. The browser should refresh.
   * - CSS or an image under ``docs/_static/``
     - Save and wait for rebuilding. Hard-refresh if the browser retains the old asset.
   * - ``conf.py``, a template, or Python code under ``docs/_ext/``
     - Restart ``make start`` so configuration, imported extensions, and templates are reloaded.
   * - ``docs-requirements.txt``, ``package.json``, or ``package-lock.json``
     - Restart ``make start``. Keep npm declarations and the lockfile consistent.
   * - Firmware headers/source or ``payload_controller/Doxyfile``
     - Restart the command to regenerate C++ XML before rebuilding the reference.
   * - Python code outside ``docs/``
     - Restart with ``SKIP_AUTOAPI_BUILD=0`` to regenerate its API pages.

The preview watches ``pages/``, ``_static/``, ``_templates/``, ``_ext/``, and
``conf.py``. It does not watch all firmware/ROS source trees or run Doxygen on
every page save. Read the terminal's warnings if the rendered page is unexpected.

If you still see older content
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Check that the URL uses the port printed by the preview you just started.
2. Stop an older preview and restart with ``make start`` from ``docs/``.
3. Wait for the build to finish, then hard-refresh the browser.
4. If Mermaid is missing, check Node is version 22 or newer and inspect the npm/
   Python installation errors in the terminal. Both dependency sets are needed.
5. For a C++ API change, check that Doxygen ran and did not fall back to older XML.

You can check whether the current source contains a Mermaid diagram by looking
for ``.. mermaid::`` in the page. The browser should load Mermaid from the site's
``_static/mermaid/`` files, rather than a Mermaid CDN.

Where files belong
------------------

.. list-table:: Source layout
   :header-rows: 1
   :widths: 38 62

   * - File or folder
     - Purpose
   * - ``docs/pages/index.rst``
     - Homepage and the three main navigation sections.
   * - ``docs/pages/<section>/index.rst``
     - Section overview and links to its child pages.
   * - ``docs/pages/**/*.rst`` or ``*.md``
     - Handwritten guides. Markdown is supported through MyST Parser.
   * - ``docs/_static/``
     - Maintained styles, logos, screenshots, and other assets.
   * - ``docs/_ext/``
     - Python extensions for contributor cards and page-status labels.
   * - ``docs/conf.py``
     - Theme, extensions, reference generation, and diagram settings.
   * - ``docs/docs-requirements.txt``
     - Python dependencies, including ``sphinxcontrib-mermaid``.
   * - ``docs/package.json`` and ``docs/package-lock.json``
     - Declared and locked Mermaid/D3 browser dependencies.
   * - ``docs/build/``
     - Generated output and prepared browser assets; not the source to edit.

Doxygen/Breathe supply the C++ reference; AutoAPI supplies the Python reference.
Write learning explanations alongside them: function declarations alone do not
explain how the system works.

Write and connect a page
------------------------

A short reStructuredText (RST) page can look like this:

.. code-block:: rst

   :doc-status: evolving
   :status-reason: Hardware integration is still changing.

   My Guide
   ========

   Explain what this subsystem does and what it connects to.

   First Steps
   -----------

   Explain a new term briefly before using it in instructions.

   .. note::

      A useful observation or limitation.

   .. code-block:: bash

      make linux

RST uses indentation and blank lines to group content. Directives such as
``.. note::`` add structured elements. Keep heading underlines at least as long
as their titles. In Markdown, use ``#`` / ``##`` headings and fenced code blocks.

For example, create ``docs/pages/pc/my-guide.rst``, then add ``my-guide`` to the
existing ``toctree`` in ``docs/pages/pc/index.rst``:

.. code-block:: rst

   .. toctree::
      :maxdepth: 2

      my-guide

Paths in that tree are relative to the parent page and omit the file extension.
Link between pages in RST with ``:doc:`My guide <my-guide>```; in Markdown use
``[My guide](my-guide.rst)`` or the corresponding ``.md`` filename.
For images, an RST page directly under ``pages/`` can use:

.. code-block:: rst

   .. image:: ../_static/my-screenshot.png
      :alt: Describe what the screenshot helps the reader see.

Adjust the relative path for deeper pages. Give each section/subsystem an overview
that explains its inputs, outputs, and interactions before its implementation.
Build explanations from plain-language ideas to code details; define unfamiliar
terms and give units, command locations, expected results, and current limits.
Link to related guides and source code. Update the
:doc:`documentation map <getting-started/organization>` when adding a learning path.

Drawing system diagrams
-----------------------

Use **Mermaid**, a text-based diagram language, for system interactions. This keeps
the diagram editable beside the guide. For an RST page:

.. code-block:: rst

   .. mermaid::

      flowchart LR
          sensor["Encoder · measured speed"] --> controller["Speed controller"]
          controller --> motor["Motor · applied effort"]

For Markdown, MyST's directive fence provides the equivalent:

.. code-block:: markdown

   ```{mermaid}
   flowchart LR
       sensor["Encoder · measured speed"] --> controller["Speed controller"]
       controller --> motor["Motor · applied effort"]
   ```

The preview supports pan, zoom, and fullscreen. Label arrows with their meaning
or units. Keep the overview small and put detail on subsystem pages. Use text
labels as well as colors. The :doc:`pc/overview` uses blue for measurement, purple
for computation, green for communication, orange for actuation, and gray for the
physical system. Check diagrams in both light and dark modes, including edge labels.

``make start`` installs the locked Mermaid/D3 versions and serves them locally.
If changing a browser dependency, update its declaration and regenerate the lockfile
with npm; do not hand-edit installed ``node_modules`` files or prepared build assets.

Marking pages in progress
-------------------------

WIP and human review are separate labels. **Yellow with WIP** means the page or
system is likely to change. It can still contain useful, detailed instructions.
Set this metadata at the top of an RST page, before the title:

.. code-block:: rst

   :doc-status: evolving
   :status-reason: Hardware acquisition is incomplete; simulation is implemented.

For Markdown, use YAML front matter:

.. code-block:: markdown

   ---
   doc-status: evolving
   status-reason: Hardware acquisition is incomplete; simulation is implemented.
   ---

The status extension adds a yellow sidebar link, a banner explaining the reason,
and an entry in the documentation map's WIP list. Status applies to that page,
not automatically to its children. Remove it when its stated uncertainty is resolved.

Recording a human review
------------------------

Every page defaults to **Not human verified**, including generated API pages.
A successful build, assistant edit, or automated check does not count as a human
review. After a club member checks a page's explanation and relevant commands,
record their actual name/handle and review date in RST metadata:

.. code-block:: rst

   :human-verified: true
   :verified-by: actual-reviewer-handle
   :verified-on: YYYY-MM-DD

For Markdown:

.. code-block:: markdown

   ---
   human-verified: "true"
   verified-by: actual-reviewer-handle
   verified-on: "YYYY-MM-DD"
   ---

Replace the placeholders: the verified label requires a true flag, a nonempty
reviewer, and a valid date such as year-month-day in ISO format. ``YYYY-MM-DD``
itself is not a valid date. Keep WIP metadata too if the reviewed page describes
unfinished/changing behavior. Review applies only to that page, not every linked page.

If later edits have not been reviewed, remove the review fields until the next
review. Git records those changes; the fields are a team-maintained record rather
than an identity-checking system. Do not edit generated API pages directly to
record a review; their source is regenerated. The label remains unverified unless
a review is recorded through maintained page metadata.

Before opening a pull request
-----------------------------

Preview the pages you changed, check links/diagrams and the terminal's warnings,
and confirm commands say which directory to run from. Keep status honest and
update the map/navigation when needed. Commit maintained source and dependency
lockfile changes; leave environments, ``node_modules``, and build output out.
Follow :doc:`tutorials/making-first-contrib` for the branch, commit, and PR workflow.

Further reading
---------------

* `Sphinx RST guide <https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html>`_
* `MyST syntax guide <https://myst-parser.readthedocs.io/en/latest/syntax/syntax.html>`_
* `Mermaid flowchart guide <https://mermaid.js.org/syntax/flowchart.html>`_
* `Shibuya writing guide <https://shibuya.lepture.com/writing/>`_

Mark key reading pages
----------------------

Use a blue **Key** marker for architecture and behavior explanations that a new
contributor should prioritize for a subsystem. Do not mark installation, setup,
vocabulary, or exhaustive API pages merely because they are useful. In RST, put
this metadata before the title, adjacent to any existing page metadata:

.. code-block:: rst

   :doc-priority: key
   :key-reason: Read before changing the subsystem's data ownership and update order.

   Page Title
   ==========

For Markdown, use the equivalent front matter fields ``doc-priority: key`` and
``key-reason: ...``. The marker adds a blue sidebar badge and an explanatory page
banner; it also adds the page to the Documentation Map's generated key-reading
list. Removing the metadata removes the marker on rebuild. Update the project's
short reading path when changing what contributors should read first.

Priority is independent of ``doc-status: evolving`` and human-review metadata.
A Key page can still be WIP and Not human verified; yellow WIP highlighting is
retained alongside the blue Key badge. See :doc:`getting-started/organization`
for the complete color key.
