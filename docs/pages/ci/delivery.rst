:doc-status: evolving
:status-reason: ARM fallback references a missing dependency-install script and obsolete launch paths; reconcile these before using it for deployment. Docs deployment is described separately.

Docs Deployment and ARM Artifacts
=================================

The repo delivers three different outputs: the Docker environment described in
:doc:`docker`, the documentation website, and an ARM runtime archive. Publishing
one does not publish the others.

Documentation delivery
----------------------

``deploy-docs.yml`` runs on a push to ``main`` or a manual dispatch. Its reusable
composite action, ``.github/actions/build-docs/action.yaml``, performs:

1. Configure Pages metadata when ``deploy`` is true.
2. Set up uv/Python 3.12 and Node 22.
3. Create a virtual environment and install ``docs/docs-requirements.txt``;
   install Doxygen through apt.
4. Source ``dev_env.sh`` and the Python environment, then run ``make html`` in
   ``docs/``. The Make prerequisites prepare pinned local Mermaid/D3 assets and
   generate the C++ API XML before Sphinx builds the site.
5. With ``deploy: true``, upload ``docs/build/html`` as a Pages artifact and deploy
   it through ``actions/deploy-pages``.

Merge Build uses the same action with ``deploy: false``: it builds the site for
validation without publishing it. The deployment workflow grants ``pages: write``
and ``id-token: write`` and names the ``github-pages`` environment. Pages runs
share a concurrency group with ``cancel-in-progress: false`` to avoid interrupting
an active deployment. Environment approvals or repository Pages settings can
still affect delivery.

If a PR build succeeds but the public site is old, check the **Deploy Docs** run
for the main commit, then the Pages deployment/environment. Local preview steps
and stale-browser troubleshooting are in :doc:`../docs-contribution`.

ARM Fallback delivery
---------------------

``arm-fallback.yml`` is a manual workflow on ``ubuntu-22.04-arm`` using the
``ros:jazzy-ros-base`` container. The runner label identifies the host; the
container supplies the Jazzy userspace. This is an ARM64 build, not the nightly
x86-64 image.

It is configured to check out recursive submodules, install dependencies using
``install_ros_ci_deps.sh``, and run ``build_arm_artifact.sh``. That script builds
shared interfaces and ``vehicle_common``, then ``payload`` and the ROS
``payload_controller`` with simulation off and GPIO on, then ``tools`` and
``uav``. It packages the install tree through ``package_arm_artifact.sh`` and
uploads ``ros2-build-<short-SHA>.tar.gz`` under a similarly named Actions artifact,
retained for 30 days. No unit-test or simulation-test step is present in this
workflow, and it does not automatically install the archive on a Pi.

The packager writes ``install/BUILD_INFO.txt``, stages ``install/`` plus available
deployment/hardware scripts, and attempts to replace the UAV launch configuration
with ``launch_params_hardware.yaml``.

.. warning::

   The workflow first calls ``ros_ws/scripts/ci/install_ros_ci_deps.sh``, which
   is absent in this checkout. The current packager also references
   ``src/common/uav/launch/launch_params_hardware.yaml``, which is absent in this
   checkout. Its generated instructions also mention ``fleet.launch.py``, which
   is absent here. The packaging path therefore needs reconciliation before it
   can be treated as a working release procedure. Inspect the workflow result and
   archive contents rather than assuming its generated deployment instructions
   match the current launch files.

To inspect a downloaded archive locally without launching hardware:

.. code-block:: bash

   tar -tzf ros2-build-<short-SHA>.tar.gz

Replace ``<short-SHA>`` with the actual filename value. Confirm the target is
ARM64, inspect ``BUILD_INFO.txt``, and check runtime dependencies on the target
machine before using an install tree. A compiled workspace does not contain an
entire operating system or all apt-installed libraries.

Issue automation
----------------

``issue.yml`` is administrative automation rather than a build. For opened or
edited issues it extracts the form's Priority (Low, Medium, High, or Urgent),
looks up the organization's Priority field, and writes that value using
``PROJECT_TOKEN``. Missing form values, field access, or token permissions can
fail this workflow independently of code quality.

Sources: `docs action <https://github.com/pennaerial/monorepo/blob/main/.github/actions/build-docs/action.yaml>`_,
`ARM Fallback <https://github.com/pennaerial/monorepo/blob/main/.github/workflows/arm-fallback.yml>`_,
`ARM packager <https://github.com/pennaerial/monorepo/blob/main/ros_ws/scripts/ci/package_arm_artifact.sh>`_.
