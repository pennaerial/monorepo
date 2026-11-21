C++ API Reference
=================

An **API** is the set of functions and types code offers to callers. This section
lists exact C++ declarations for lookup. It is not the starting lesson: use the
linked subsystem guides to learn what those functions do, and :doc:`../glossary`
for unfamiliar words. A **header** declares an interface; the ``.cpp`` source
implements it. Doxygen extracts the declarations/comments, and Breathe inserts
them into the documentation site.

Use these generated declarations to look up function signatures after reading
:doc:`../overview`. They describe the firmware in root ``payload_controller/``;
they are not the similarly named ROS package. Doxygen reads source comments and
produces XML, and Breathe embeds the result here.

.. toctree::
   :maxdepth: 1

   runtime
   drivers
   backends
   communications
   estimator
   controllers
   helpers

To regenerate, run ``make html`` or ``make serve`` from ``docs/`` after sourcing
``dev_env.sh`` and activating the docs environment. Doxygen must be installed.
Generated declarations do not certify that a backend is implemented; consult
the corresponding learning page for current limitations.
