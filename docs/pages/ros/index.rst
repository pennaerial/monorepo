:doc-status: evolving
:status-reason: Existing ROS guides are grouped here; their educational refactor is deferred.

ROS
===

ROS nodes coordinate vehicle behavior through messages. A mission selects a
sequence of actions, modes implement individual behaviors, and the mode manager
controls which behavior runs. The Python API reference provides class and function
lookups. This section preserves the existing guides while the embedded learning
path is developed.

For the embedded side of the connection, see :doc:`../pc/dds_client/index`.
For laptop prerequisites, see :doc:`../installation/index`.

.. toctree::
   :maxdepth: 2

   Concepts and System Interactions <../concepts/index>
   Create a Mode <../tutorials/create-first-mode>
   Create a Mission <../tutorials/create-mission>

.. docs-python-api::
   :maxdepth: 2

   Python API Reference <../autoapi/index>
