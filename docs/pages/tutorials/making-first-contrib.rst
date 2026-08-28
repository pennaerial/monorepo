Making Your First Contribution
==============================

.. rst-class:: lead

    A simple guide on making your first contribution to PennAiR Software. You will learn how to add yourself to the **Active Contributors** list
    on our docs landing page, and also learn our git workflow.

----

**NOTE**: This tutorial assumes that you have already cloned monorepo and are checked out to its most up-to-date main branch.

Setting Up Docs Environment
```````````````````````````

Making changes to the documentation requires you to be able to run the docs locally. This lets you verify that your changes are working properly.

Follow the **Running Docs Locally** section in our guide for :doc:`Contributing to Docs <../docs-contribution>`. Make sure that run the
``SKIP_AUTOAPI_BUILD=1 make serve`` command for faster builds.

Creating a Branch
`````````````````

Before making any changes, create a new branch for your contribution. Use the following naming convention, replacing ``<github-username>`` with your GitHub username:

.. code-block:: bash
    :caption: Bash

    git checkout -b user/<github-username>/add-contributor

This keeps contributor changes organized and makes it easy to identify the purpose of your branch.

Adding Yourself to the Contributors
````````````````````````````````````

Create a new YAML file in the contributors directory using your GitHub username as the filename:

.. code-block:: text

    monorepo/docs/pages/contributors/<github-username>.yaml

Add the following to the file, replacing <github-username> with your GitHub username:

.. code-block:: yaml

    github-username: <github-username>
    role: member

.. note::

    The ``role`` should be set to member unless you have been designated a **lead** role.

Then, go to the locally running docs website, usually at `http://localhost:8000 <http://localhost:8000>`_. You should now be able to
see yourself in the **Active Contributors** list.

.. note::

    The contributor display is implemented as a custom **Sphinx** extension in ``docs/_ext/pennair_contributors.py``.

    It reads contributor YAML files from the specified directory, fetches contribution counts from GitHub's API,
    and dynamically generates the contributor grid. Contributors are sorted by descending contribution count.

    The extension also supports filtering by active members or alumni. Contributors with a **lead** role are marked with a bird icon next to their name.

Committing and Pushing Your Changes
````````````````````````````````````
Once you have created your contributor file, stage it with Git:

.. code-block:: bash
    :caption: Bash

    git add docs/pages/contributors/<github-username>.yaml

Then commit your changes with the following commit message:

.. code-block:: bash
    :caption: Bash

    git commit -m "docs: add <github-username> as contributor"

Replace ``<github-username>`` with your GitHub username. For example:

Finally, push your branch to GitHub:

.. code-block:: bash
    :caption: Bash

    git push -u origin user/<github-username>/add-contributor

After pushing, open a pull request on GitHub from your branch into ``main``.

.. note::

    More about pull requests can be found on the `GitHub Docs <https://docs.github.com/en/pull-requests/how-tos/create-pull-requests/creating-a-pull-request>`_.
