Troubleshooting and Changing CI
===============================

A failure is easiest to diagnose at the first layer that broke. Keep the parent
commit, image digest, and submodule versions with your notes when a failure is
hard to reproduce.

Find the responsible layer
--------------------------

.. list-table:: Common symptoms
   :header-rows: 1
   :widths: 28 36 36

   * - Symptom
     - First place to inspect
     - Next action
   * - Ruff or C++ formatting fails
     - Static Checks command output
     - Run the matching command in :doc:`checks`; review the resulting diff.
   * - Workflow syntax or expression error
     - actionlint output and workflow YAML
     - Correct the reported line, then run actionlint locally.
   * - Job fails before its first step
     - Runner/container preparation logs
     - Check image availability, GHCR access, and runner architecture.
   * - Missing Python/apt dependency
     - Quick Install Dependencies logs and manifests
     - Declare it in the owning package or root dependency configuration.
   * - ROS compiler/import error
     - Build ROS Packages; ``ros_ws/log/latest_build/`` locally
     - Check the first package failure, its dependencies, and sourced overlays.
   * - Third-party build unexpectedly reruns
     - ``build_ros.sh`` COMMIT_TAG messages
     - Compare current submodule SHA with the cached package tag.
   * - Simulation timeout
     - Run Sim Tests and test logs
     - Check PX4/Gazebo startup, Agent port, model resources, and ground-station heartbeat.
   * - Tests appear to finish but job fails
     - Final ``colcon test-result --verbose``
     - Read the failing test report; a launcher exit alone does not prove success.
   * - ESP toolchain absent
     - Ensure ESP-IDF and target build logs
     - Check EIM config, activation path, direnv, and target-specific SDKCONFIG.
   * - Docs import/Doxygen/Mermaid failure
     - Build Docs dependency and Sphinx logs
     - Match :doc:`../docs-contribution`; check Node, Python dependencies, and XML generation.
   * - ARM packaging cannot copy a file
     - Package ARM runtime artifact
     - Compare source paths with the current launch layout; see :doc:`delivery`.

Change the source of the behavior
---------------------------------

* **ROS package dependency:** edit its ``package.xml`` for rosdep/build/runtime
  dependencies. Add Python packages not supplied through rosdep to the root
  ``pyproject.toml`` as appropriate; keep package install requirements coherent.
* **Extra image apt package or managed submodule:** edit ``ci/ci.conf``.
* **Cached ROS submodule:** edit ``ros_ws/scripts/ci/ros_ci.conf`` and verify its
  source location matches the script's ``src/<group>/<package>`` lookup.
* **Heavy prebuild:** edit ``ci/image_build.sh`` or full install steps in
  ``ci/install_deps.sh``; quick PR installs do not rebuild those layers.
* **Test discovery:** inspect ``.aliasrc`` and the package's test configuration.
  A new ``sim_test_*.py`` must belong to a selected Python package and work with
  the simulation environment; merely placing it elsewhere does not run it.
* **New validation job:** add its change filter, job, and dependencies in
  ``mergebuild.yml``. Add it to both ``merge_build_status.needs`` and the shell
  result map, or the aggregate result will not account for it.
* **Docs tool:** update the committed docs dependency file and lockfile where
  relevant; keep local startup and the composite action aligned.

Path filters are part of correctness. If a new root-level file affects a build,
include it in the relevant filter. Test both the changed-file case and the
unrelated-change case by inspecting PR jobs. For any renamed required check,
coordinate the corresponding GitHub branch rules; the YAML does not update them.

Permissions and reruns
----------------------

Merge Build has read access to contents/packages. Nightly additionally writes
packages, contents, and PRs. Docs deployment has Pages/OIDC permissions. Avoid
copying a publishing workflow's permissions into a read-only validation job.
Fork PRs and package visibility can affect credential availability; a failure
to pull an image is not a compiler failure.

From GitHub's **Actions** tab, choose a workflow and use **Run workflow** for a
manual dispatch where enabled. Select the intended branch and inspect its run.
For an existing run, rerun failed jobs after diagnosing the cause. A rerun can
pull a newer ``latest`` image or newer managed submodules, so a later success
may reflect a changed environment as well as changed code.

Scripts and boundaries to know
------------------------------

The active Merge Build uses ``build_ros.sh`` and the ``.aliasrc`` test helpers.
Older ``validate_ros_workspace.sh`` and ``validate_sim_headless.sh`` use other assumptions,
including older image paths and test filenames; they are not called by the
current workflows. ``.github/deprecated/`` is also not an active workflows
folder. Read callers before using a script as the source of CI behavior.

Before proposing a CI change, run its applicable local checks, review its path
filters and permissions, and explain which new behavior the check covers. A
simulation success should be reported separately from a build-only success.

Source: `ROS CI scripts <https://github.com/pennaerial/monorepo/tree/main/ros_ws/scripts/ci>`_
and `workflow inventory <https://github.com/pennaerial/monorepo/tree/main/.github/workflows>`_.
