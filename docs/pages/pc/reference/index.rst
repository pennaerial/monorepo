C++ API Reference
=================

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
