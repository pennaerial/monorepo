Documentation Map
=================

This is the maintained map of the website. Start with a section's **system
overview** to see its inputs, outputs, and connections, then follow its subsystem
pages. Read a section in order to learn; use the source links and API reference
when looking up a particular function.

Status color key
----------------

* **Yellow, with “WIP” text:** the page is in progress or describes a system with
  incomplete implementation, provisional calibration, or likely interface changes.
  Its banner explains what remains uncertain. Yellow does not mean the page is empty.
* **Not human verified:** the default for every page without a recorded review.
  The sidebar label and gray banner mean nobody has recorded a human check yet.
* **Human verified:** a club member has reviewed that particular page and recorded
  their name and review date. This does not mean every linked page was reviewed,
  and a correct explanation can still describe unfinished software.

Review and WIP are separate: a human-reviewed page may still be yellow if the
system is changing. No page is treated as verified just because it has normal
colors, passes a build, or was checked by an automated tool. All pages in this
refactor remain unverified until a person reviews them.

Status appears on sidebar links, on the page itself, and in the live list below.
The text label makes the distinction usable without relying on color. Status is
maintained in each page's metadata; see :doc:`../docs-contribution`.

Getting Started
---------------

:doc:`index` explains how the monorepo's projects interact.

* :doc:`../installation/index`: laptop tools, Ubuntu, macOS VM, Windows/WSL,
  and the Ubuntu update guide linked from the Ubuntu instructions.
* :doc:`../hardware/index`: Raspberry Pi setup, Ethernet, and SSH.
* :doc:`../tutorials/making-first-contrib`: branch, edit, preview, commit, and PR.
* :doc:`../development-guide`: development conventions and review workflow.
* :doc:`../docs-contribution`: local preview, writing pages, and status markers.

Payload Controller ESP
----------------------

:doc:`../pc/index` is the project entry point; :doc:`../pc/overview` traces one
complete pass through the system.

* :doc:`../pc/glossary`: short explanations of the tools, signals, and concepts used here.
* :doc:`../pc/getting-started`: prerequisites, target switching, flashing,
  Gazebo, and troubleshooting.
* :doc:`../pc/project-organization`: directories and component connections.
* :doc:`../pc/components`: adding/changing components and dependencies, including Eigen.
* :doc:`../pc/build-system`: Make, ESP-IDF, CMake, configuration, and generated files.
* :doc:`../pc/dds_client/index`: Agent, topics, publishing, callbacks, and message generation.
* :doc:`../pc/sensors/index`: common driver contracts, then IMU, motors,
  encoders, and camera; each has separate SITL and hardware pages.
* :doc:`../pc/payload`: the Payload class, update ordering, and sample ownership.
* :doc:`../pc/estimation/index`: start with the non-expert ES-EKF tutorial and
  worked correction example, then follow equations/code and update-loop integration.
* :doc:`../pc/control/index`: control overview, feedforward, PID, and PWMController.
* :doc:`../pc/runtime-helpers`: locks, sample counters, logging, and left/right ownership.
* :doc:`../pc/simulation`: simulator startup, worker threads, and message conversion.
* :doc:`../pc/development-tools`: static analysis and the current test limitations.
* :doc:`../pc/code-map`: file-by-file reading index for the C++ project.
* :doc:`../pc/reference/index`: generated C++ API grouped by subsystem.

ROS
---

:doc:`../ros/index` groups the existing mission, mode, and Python reference
material. Its learning-path refactor is deferred; the Payload Controller and
Getting Started sections are the current priority.

Pages under active development
------------------------------

This list is generated from page metadata, so it stays synchronized with the
navigation colors.

.. docs-status-list::

Maintaining this map
--------------------

When adding a subsystem, give its index page a short interaction overview, a
learning order, and a table of contents. Link it from its parent and update this
map if its role changes. Keep existing page URLs where practical so shared links
continue to work. Explain current behavior separately from planned work.
