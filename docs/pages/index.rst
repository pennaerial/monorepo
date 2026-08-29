:layout: landing



.. image:: ../_static/penn-air-full-logo.png
    :alt: PennAiR Logo
    :align: center
    :width: 600px

PennAiR Monorepo
==================

.. rst-class:: lead

    Welcome to the Monorepo for the Penn Aerial Robotics Software Team! This repository is designed
    to centralize all code and documentation for our projects.

    Click **Get Started** or any of the Quick Links below.

.. container:: buttons

    :doc:`Get Started <installation/index>`
    `GitHub <https://github.com/pennaerial/monorepo>`_

Quick Links
-----------

.. grid:: 1 1 2 3
    :gutter: 2
    :padding: 0

    .. grid-item-card:: :iconify:`lucide:download` Installation
        :link: installation/index
        :link-type: doc

        Installation Guide for Linux/macOS. Includes ROS 2, PX4, Gazebo Simulation, and ESP-32 toolchain for payload firmware.

    .. grid-item-card:: :iconify:`lucide:book-open` Tutorials
        :link: tutorials/index
        :link-type: doc

        Beginner Tutorials for monorepo development. Includes making your first contribution, creating modes, mission, etc.

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: GETTING STARTED

   installation/index
   tutorials/index
   docs-contribution

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: SAE 2025 WORKSPACE

   Python API Reference <autoapi/index>

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: PAYLOAD CONTROLLER

   C++ API Reference <pc_api/index>

Active Contributors
-------------------

.. pennair-contributors::
    :path: contributors
    :repo: pennaerial/monorepo
    :role: active

Past Contributors (Alums)
-------------------------

.. pennair-contributors::
    :path: contributors
    :repo: pennaerial/monorepo
    :role: alum
