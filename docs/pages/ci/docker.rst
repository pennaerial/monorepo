Docker Images and the Nightly Build
===================================

An image is a packaged filesystem and environment; a container is a running
instance of it. CI uses an image to avoid reinstalling and compiling PX4 and
other large dependencies for every pull request. The image contains a repository
checkout too, so validation must update that checkout to the requested commit.

What the image contains
-----------------------

``ci/Dockerfile`` starts from ``ros:jazzy-ros-base`` (Ubuntu 24.04), sets
``ROS_DISTRO=jazzy`` and ``GZ_VERSION=harmonic``, and defines the same repository
paths used by ``dev_env.sh``, rooted at ``/root/monorepo``.

Its build sequence is:

1. Install Git and clone the repository using the ``BRANCH`` build argument
   (default ``main``).
2. Run ``ci/update_submodules.sh``: initialize recursively, then move the managed
   submodules to their remote default branches.
3. Run the full ``ci/install_deps.sh``: install uv, PX4 host dependencies,
   rosdep dependencies, ``ci/ci.conf`` apt extras, ESP-IDF, and root
   ``pyproject.toml`` Python dependencies.
4. Run ``ci/image_build.sh``: build ``Dependencies`` modules, ``make px4_sitl``,
   and a clean build of the third-party ROS packages.

``ros_ws/scripts/ci/ros_ci.conf`` identifies the cached ROS packages as
``px4_msgs`` and ``camera_ros``. PR-time ``build_ros.sh`` compares their Git SHAs
with ``build/<package>/COMMIT_TAG``. Quick mode reuses these packages only when
they match; other workspace packages get fresh package build/install directories.
Quick mode is therefore not a normal incremental build for club-owned packages.
``--build-mode clean`` removes the workspace's entire build and install trees.

Managed versus pinned submodules
--------------------------------

``ci/ci.conf`` currently manages ``Dependencies``, ``PX4-Autopilot``, and
``ros_ws/src/common/camera_ros``. Their latest default-branch commits can be
selected by CI independently of the parent repository's recorded pins. Other
submodules stay at the recursively recorded commits.

This matters when reproducing a failure: record both the parent SHA and the
submodule SHAs. Running ``ci/update_submodules.sh`` on a development checkout
changes those submodule checkouts; use a disposable clone when reproducing CI's
exact update procedure. It is more than an initialization command.

How Nightly decides to rebuild
------------------------------

.. mermaid::

   flowchart LR
       accTitle: Nightly image refresh
       accDescr: Remote commit metadata produces a combined hash. A changed hash triggers image publication and a managed submodule bump request.
       refs["Main SHA + managed remote HEADs"] --> hash["Sorted input → 12-character hash"]
       hash --> compare{"Matches latest image label?"}
       compare -->|yes| skip["Skip build"]
       compare -->|no| build["Docker Buildx build"]
       build --> push["Push hash tag and latest"]
       push --> bump["Open submodule bump PR if pins changed"]

``compute_image_metadata.sh`` queries remote refs and writes ``ci/version.json``.
Nightly compares its combined tag with the ``pennaerial.combined_tag`` label on
``latest``. When they differ, Buildx builds with context ``ci`` and
``CACHEBUST_CHECKOUT`` set to the combined tag. The Dockerfile clones the repo
itself; it does not copy your local working tree into the image.

A successful build publishes both ``:<combined_tag>`` and ``:latest``, with
revision/submodule labels. Nightly then checks out the measured managed SHAs in
its checkout, opens a bump PR when pins changed, and requests auto-merge using
squash. Actual merging still depends on repository settings and checks. A
separate cleanup job keeps at least ten versions while deleting untagged
versions for the configured ``monorepo`` and ``sae-ros2-ci`` packages.

The metadata query and Docker clone happen separately against remote refs.
They are not an atomic snapshot; a branch advancing during the run can create
drift. ``latest`` is also mutable. Preserve a pulled image's digest and observed
checkout SHAs when investigating an environment-dependent failure.

Inspect or use the CI environment locally
-----------------------------------------

With Docker running on an x86-64 Linux host, from any directory:

.. code-block:: bash

   docker pull ghcr.io/pennaerial/monorepo:latest
   docker image inspect ghcr.io/pennaerial/monorepo:latest \
     --format '{{json .RepoDigests}}'
   docker image inspect ghcr.io/pennaerial/monorepo:latest \
     --format '{{json .Config.Labels}}'
   docker run --rm -it ghcr.io/pennaerial/monorepo:latest bash

If pulling is denied, authenticate to GHCR with credentials permitted to read
that package; access depends on its GitHub settings. The current nightly build
has no multi-platform matrix; ARM Fallback uses a separate ARM runner instead.
A different host architecture may require emulation.

Inside the container, inspect its existing checkout before running checks:

.. code-block:: bash

   cd /root/monorepo
   git rev-parse HEAD
   git submodule status --recursive
   source /opt/ros/jazzy/setup.bash
   source dev_env.sh
   bash ros_ws/scripts/ci/build_ros.sh --build-mode quick
   source ros_ws/install/setup.bash

This builds the image's checkout. It does not include your uncommitted laptop
edits. To reproduce a committed branch, fetch and check it out in this disposable
container first, then follow the update/install/build sequence in :doc:`checks`.
Exiting a ``--rm`` container discards its changes. Do not mount your working tree
over ``/root/monorepo`` when trying to preserve the image's cached build trees.

To build a separate local image from a **remote committed branch**, from the
repository root:

.. code-block:: bash

   docker build --build-arg BRANCH=main \
     --build-arg CACHEBUST_CHECKOUT=local-rebuild \
     -f ci/Dockerfile -t pennair-ci-local ci

Change the cache-bust value for a later fresh clone; reusing it may reuse Docker
layers. This is a large dependency build, not a fast way to preview docs. It
creates only a local image and does not push or deploy anything.

Sources: `Dockerfile <https://github.com/pennaerial/monorepo/blob/main/ci/Dockerfile>`_,
`Nightly <https://github.com/pennaerial/monorepo/blob/main/.github/workflows/nightly.yml>`_,
`CI scripts <https://github.com/pennaerial/monorepo/tree/main/ci>`_.
Further reading: `Docker images and containers <https://docs.docker.com/learn/content/what-is-a-container/>`_.
