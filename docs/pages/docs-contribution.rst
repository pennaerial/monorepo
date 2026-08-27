Contributing to Docs
=====================

.. rst-class:: lead

   Learn how to contribute to our documentation.

----

This guide will teach how to run the documentation website locally and cover the basics of
writing documentation in reStructuredText (.rst) or Markdown (.md).


Running Docs Locally
````````````````````

Monorepo uses `Sphinx <https://www.sphinx-doc.org/en/master/>`_ to generate this documentation website, using the **Shibuya** theme.

First, create a virtual environment and install all required dependencies.


.. tab-set::
    :class: outline

    .. tab-item:: :iconify:`devicon:pypi` pip

        .. code-block:: bash

            cd sphinx
            python3 -m venv venv
            source venv/bin/activate
            pip install -r docs-requirements.txt
            sudo apt install doxygen # required for generating C++ API Reference

    .. tab-item:: :iconify:`material-icon-theme:uv` uv

        .. code-block:: bash

            cd sphinx
            uv venv venv
            source venv/bin/activate
            uv pip install -r docs-requirements.txt
            sudo apt install doxygen # required for generating C++ API Reference

Then, run the command for serving locally:

.. code-block:: bash
    :caption: Bash

    make serve

.. note::

   We use ``sphinx-autoapi`` to generate the Python API Reference. ``sphinx-autobuild`` is used to
   automatically rebuild the local website every time a source file in ``docs/`` is changed.
   To prevent a slow autoapi build on every change, so you can set the environment variable
   ``SKIP_AUTOAPI_BUILD=1`` when running ``make serve`` to skip this step. This results in faster
   iteration when hand-writing documentation, which is the majority of cases.

You should then be able to see the landing page at `<http://localhost:8000/>`_.


Writing Documentation
`````````````````````

Sphinx natively renders reStructuredText (RST), which is basically just Markdown but more powerful.

Here is a quick taste of RST syntax

.. code-block:: rst
   :caption: example.rst

    Page Title
    ==========

    Section Heading
    ---------------

    Some **bold text**, *italic text*, and ``inline code``.

    - A bullet point
    - Another bullet point

    .. note::

       This is an admonition, useful for callouts.

    .. code-block:: python

        print("a code block")

    .. toctree::
       :maxdepth: 2
       :caption: This is a table of contents tree

       each-page
       links-to-a-different
       docs-source-file
       md-or-rst

RST uses indentation and blank lines to define structure, and "directives" (the ``.. name::`` blocks)
for anything beyond plain text — tabs, notes, images, code blocks, and so on.

Sphinx also supports using markdown files through ``MyST-Parser`` to translate markdown into RST.

For in-depth resources:
 - `reStructuredText Overview <https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html>`_
 - `MyST-Parser Syntax Guide <https://myst-parser.readthedocs.io/en/latest/syntax/syntax.html>`_
 - `Shibuya Writing Guide <https://shibuya.lepture.com/writing/>`_ - go to various pages and click *Edit this page* to see good RST examples
