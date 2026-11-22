:doc-priority: key
:key-reason: Read before changing checks or diagnosing a PR failure: path filters and actual test coverage matter.

Automatic Checks and Local Reproduction
=======================================

Start with the failing job in the pull request's **Checks** tab. Read the first
failing command and its error, rather than the final summary alone. Static
checks inspect code; build checks compile it; tests exercise selected behavior.
Each can catch problems the others miss.

Static Checks
-------------

The path filters decide which steps run, but the selected commands generally
check the whole configured scope, not only the changed lines.

.. list-table:: Actual checks in ``static-checks.yml``
   :header-rows: 1

   * - Changed paths
     - Commands and scope
   * - Any ``.py`` or ``pyproject.toml``
     - Ruff 0.16.7: ``ruff format --check --diff .`` then ``ruff check .``.
   * - ``.github/workflows/**``
     - Downloads actionlint and runs ``./actionlint -color``.
   * - C/C++ extensions ``cpp``, ``cc``, ``h``, ``hpp``, ``hh``
     - Installs Clang tools and runs ``cppfmt --check``.
   * - ``rust/**`` or ``monorepo.toml``
     - In ``rust/cli``: ``cargo fmt --check`` and ``cargo build --locked``;
       then validates ``monorepo.toml`` with the built CLI.

Ruff's configuration in ``pyproject.toml`` uses a 100-character line length and
rules E4, E7, E9, F, and I: selected syntax/style checks, undefined/unused names,
and import ordering. It excludes dependency/submodule directories listed there.
``cppfmt`` in ``.aliasrc`` scans tracked C/C++ files and uses ``.clang-format``.
The workflow installs clang-tidy but currently does not run it. Rust Clippy and
Rust tests are also not steps in this workflow.

From the repository root, with Ruff installed at the CI version and Clang tools
available, reproduce the applicable checks:

.. code-block:: bash

   uvx ruff@0.16.7 format --check --diff .
   uvx ruff@0.16.7 check .
   source dev_env.sh
   source .aliasrc
   cppfmt --check
   # If actionlint is installed on PATH:
   actionlint -color

To apply Python fixes, run ``uvx ruff@0.16.7 format path/to/changed.py`` and
``uvx ruff@0.16.7 check --fix path/to/changed.py``; inspect the diff afterward.
``cppfmt`` without ``--check`` formats every matching tracked file, so a targeted
``clang-format -i path/to/changed.cpp`` is usually easier to review.

For Rust, from ``rust/cli``:

.. code-block:: bash

   cargo fmt --check
   cargo build --locked
   cd ../..
   rust/target/debug/pennair --manifest monorepo.toml config check

Merge Build selection
---------------------

``detect_changes`` produces these filters:

* ``all``: ``ci/**``, ``.github/workflows/mergebuild.yml``, or ``dev_env.sh``.
  It enables all three validation jobs.
* ``ros_ws``: ``ros_ws/**`` enables ROS validation.
* ``payload_controller``: ``payload_controller/**`` enables the root ESP project.
* ``docs``: ``docs/**`` or ``.github/actions/build-docs/action.yaml`` enables docs.

These are the actual filters even where adjacent TODO comments are outdated.
For example, changing only root ``pyproject.toml`` selects Ruff but does not by
itself select Merge Build's ROS job. Changes to dependencies outside these
filters need deliberate validation; do not infer that a skipped job tested them.

ROS validation, step by step
----------------------------

1. Start in ``ghcr.io/pennaerial/monorepo:latest``, which already has expensive
   dependencies built. Fetch and force-checkout the workflow SHA **inside that
   container**, then run ``ci/update_submodules.sh``.
2. Run ``ci/install_deps.sh --quick`` to refresh rosdep, extra apt packages, and
   Python dependencies without rebuilding all heavy image dependencies.
3. Run ``ros_ws/scripts/ci/build_ros.sh --quick``. The script defaults to quick
   mode; its parser currently recognizes ``--build-mode quick`` rather than a
   separate ``--quick`` option. The workflow still gets quick mode by default.
4. Source ROS, the workspace, and ``.aliasrc``. Run ``ct`` for the normal test
   collection. It uses ``ctp`` to run colcon, merge JUnit results, and report
   failures through ``colcon test-result``.
5. Delete old test results, run ``ctsim``, then explicitly run
   ``colcon test-result --verbose``. ``ctsim`` selects ``ament_python`` packages
   and ``sim_test_*.py`` files. Its exit status alone is insufficient to detect
   all test failures; the workflow's final result command is essential.

The simulation step has a 30-minute limit. The basic UAV simulation test uses a
headless ground station helper and checks arming, airborne state, then landing.
It exercises the configured mission, not every flight mode or environment.

Local normal tests, after building and sourcing as in :doc:`../ros/getting-started`,
from ``ros_ws/``:

.. code-block:: bash

   source ../.aliasrc
   ctp vehicle_common uav
   # Or all discovered packages:
   ct

For simulation tests, use a separate simulator session with Gazebo, PX4 SITL,
Agent, models, and all runtime dependencies installed:

.. code-block:: bash

   source ../.aliasrc
   # Remove old results so the report covers this run:
   colcon test-result --delete-yes
   ctsim
   colcon test-result --verbose

This clears test reports, not source or build products. Simulation tests may
start their own Gazebo/PX4 processes; stop conflicting manually launched instances.

ESP and docs validation
-----------------------

The root ``payload_controller/`` job prepares ESP-IDF 6.0.2 if needed, then uses
``direnv exec . make linux`` and ``direnv exec . make esp32s3``. It currently
builds both targets without running firmware simulation or hardware tests.
Use :doc:`../pc/getting-started` to reproduce these builds on your laptop.

The docs job invokes ``.github/actions/build-docs`` with ``deploy: false``.
It installs Python 3.12 dependencies, Node 22, and Doxygen, then runs ``make html``.
Local startup and the full Python API build are explained in
:doc:`../docs-contribution`. A quick writing preview skips Python AutoAPI, so use
``SKIP_AUTOAPI_BUILD=0 ./start.sh html`` from ``docs/`` when checking the full mode.

Reading the aggregate result
----------------------------

``Merge Build Status`` waits for ROS, docs, and ESP validation and runs even
when a dependency failed. It accepts **success or skipped**, and fails for other
results. Thus a green summary can include intentionally skipped jobs. It does
not include the separate Static Checks workflow. Which checks block merging is
a GitHub repository setting, not established by this YAML alone.

New pushes cancel older runs for the same ref in Static Checks and Merge Build.
Read the newest run before trying to fix a cancellation from an older commit.

Sources: `Static Checks <https://github.com/pennaerial/monorepo/blob/main/.github/workflows/static-checks.yml>`_,
`Merge Build <https://github.com/pennaerial/monorepo/blob/main/.github/workflows/mergebuild.yml>`_,
`test helpers <https://github.com/pennaerial/monorepo/blob/main/.aliasrc>`_.
Further reading: `Ruff formatter and linter <https://docs.astral.sh/ruff/>`_.
