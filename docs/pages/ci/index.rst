CI/CD, Docker, and Automatic Checks
===================================

When you open a pull request, GitHub runs checks so reviewers can see whether
formatting, builds, and tests pass. After code reaches ``main``, other workflows
publish the documentation and maintain a reusable build image. This section
explains the checked-in configuration; repository settings such as required
branch checks and package visibility must be checked on GitHub itself.

How the systems interact
------------------------

.. mermaid::

   flowchart TB
       accTitle: CI and delivery relationships
       accDescr: Pull requests select static checks and build validations. Nightly publishes the shared container. Docs deployment and manual ARM packaging deliver separate outputs.
       pr["Pull request or main push"] --> static["Static Checks · formatting and lint"]
       pr --> paths["Merge Build · changed-path detection"]
       paths --> ros["ROS build → unit tests → simulation tests"]
       paths --> esp["Payload ESP · Linux and ESP32-S3 builds"]
       paths --> docs["Documentation build"]
       ros & esp & docs --> status["Merge Build Status"]
       nightly["Nightly · 05:00 UTC or manual"] --> image["Build and publish Docker image"]
       image -. "provides environment" .-> ros & esp
       main["Main push or manual"] --> pages["Deploy Docs → GitHub Pages"]
       manual["Manual ARM Fallback"] --> artifact["Raspberry Pi runtime archive"]

A **workflow** is a YAML file in ``.github/workflows/``. Its **jobs** run on
runners, and its **steps** invoke shell commands or reusable actions. Jobs can
run in parallel unless ``needs`` makes one wait for another. CI means continuous
integration: checking changes together. CD here includes publishing the docs
and build images; the ARM archive is delivered for a separate hardware install.

Read in this order
------------------

.. toctree::
   :maxdepth: 2

   checks
   docker
   delivery
   maintenance

Workflow inventory
------------------

.. list-table:: Checked-in workflows
   :header-rows: 1
   :widths: 23 30 47

   * - File under ``.github/workflows/``
     - Trigger
     - Result
   * - ``static-checks.yml``
     - PR, push to main, manual
     - Path-selected Python, C++, workflow, and Rust checks.
   * - ``mergebuild.yml``
     - PR, push to main, manual
     - Selected ROS, ESP, and docs builds, plus an aggregate status.
   * - ``nightly.yml``
     - Daily at 05:00 UTC, manual
     - Docker image refresh, managed-submodule bump PR, untagged image cleanup.
   * - ``deploy-docs.yml``
     - Push to main, manual
     - Builds and deploys GitHub Pages.
   * - ``arm-fallback.yml``
     - Manual
     - Builds an ARM64 runtime archive; retains the upload for 30 days.
   * - ``issue.yml``
     - Issue opened or edited
     - Copies a valid issue-form priority to the organization's Priority field.

A green check applies only to the work that ran. It does not establish hardware
behavior, validate every competition mission, or imply human review of a doc.
:doc:`checks` lists the current coverage precisely.

Source: `workflow directory <https://github.com/pennaerial/monorepo/tree/main/.github/workflows>`_.
Further reading: `GitHub Actions concepts <https://docs.github.com/en/actions/get-started/understand-github-actions>`_.
