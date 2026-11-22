Encoders in SITL
================

``Encoder_SITL`` subscribes to ``/world/<world>/model/<model>/joint_state`` and
selects ``front_left_wheel_joint`` or ``front_right_wheel_joint``. It rounds joint
position in radians into counts using 617 counts/revolution.

Velocity is computed from count change over at least **100 ms of simulation
time**. Shorter intervals update counts only. The first timestamp seeds the
window; a backwards clock reset reseeds it. A message without a timestamp can
update counts but cannot produce a new speed estimate. This avoids dividing by
unknown or negative elapsed time.

Run :doc:`../../getting-started`, then inspect:

.. code-block:: bash

   gz topic -e -t /world/sae/model/payload_2/joint_state

Verify both wheel joint names and a progressing header timestamp. Counts should
track wheel position; velocity should appear after an averaging window. At
constant speed, compare the count-derived speed to the joint's velocity field.
If only one side updates, inspect joint-name selection before changing controller gains.

This quantized/windowed measurement is deliberately different from reading Gazebo's
perfect instantaneous joint velocity. Window length trades noise against feedback
latency, which affects :doc:`../../control/pwmcontroller`.

Source: `encoder_sitl.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/encoder/encoder_sitl.cpp>`_.
