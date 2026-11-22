ROS: Missions, Vehicles, and Projects
=====================================

The ROS workspace coordinates vehicle behavior, simulation, and project-specific
algorithms. Start with the general guide to build and inspect one running
system, then use the package guides to follow its implementation. Prerequisites
live in :doc:`../installation/index`.

How the sections fit together
-----------------------------

A launch file starts the nodes and loads configuration. A mission manager runs
behaviors from a YAML mission; those behaviors call a vehicle adapter. The adapter
exchanges commands and telemetry with the autopilot or payload controller.
Simulation supplies the environment for testing these connections on a computer.

* :doc:`getting-started`: ROS basics, building/sourcing, one simulated mission,
  nodes/topics/services/parameters, and first debugging steps.
* :doc:`common`: the shared mission framework, UAV/PX4 path, launch conventions,
  simulation and utilities, with boundaries around evolving vision/planning work.
* :doc:`iarc`: trajectory/controller/mode structure for IARC (WIP).
* :doc:`sae`: payload, ROS motor controller, interfaces, and SAE ownership (WIP).

The conceptual guides and authoring tutorials below expand on missions and modes.
For build/test automation, continue to :doc:`../ci/index`; for the separate
embedded ESP controller, use :doc:`../pc/index`.

.. toctree::
   :maxdepth: 2

   getting-started
   common
   iarc
   sae
   Concepts and System Interactions <../concepts/index>
   Create a Mode <../tutorials/create-first-mode>
   Create a Mission <../tutorials/create-mission>

.. docs-python-api::
   :maxdepth: 2

   Python API Reference <../autoapi/index>
