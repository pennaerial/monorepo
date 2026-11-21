:doc-status: evolving
:status-reason: The motor model uses estimated gearbox efficiency and fixed electrical constants; calibration may change.

Motors in SITL
==============

This is the ``linux`` simulation backend. **Gazebo Transport** carries its motor
commands to the simulator. **Torque** is turning force, measured in newton-metres
(N·m); it is different from speed (rad/s) or duty (a unitless drive request).
To **publish** means send a message on a named channel.

``Motor_SITL`` publishes ``gz::msgs::Double`` torque commands on:

.. code-block:: text

   /model/<model>/joint/front_left_wheel_joint/cmd_force
   /model/<model>/joint/front_right_wheel_joint/cmd_force

It subscribes to joint state for output-shaft velocity, then estimates DC motor
torque. With duty ``u``, wheel speed ``omega_w``, gearbox ratio ``N``, and efficiency
``eta``, the implementation computes:

.. math::

   V &= \mathrm{clamp}(u,-1,1) V_b \\
   \omega_m &= N\omega_w \\
   I &= (V-k_e\omega_m)/R \\
   \tau_w &= k_t I N\eta

A spinning motor generates an opposing voltage called **back electromotive force
(EMF)**. Its value here is ``k_e * omega_m``, so it increases with motor speed.
Thus duty is not proportional to a fixed torque independent of speed. The current
constants are 6 V, 8.955 ohms, ``k_t=k_e=0.002324`` in their respective SI units,
ratio 100.37, and estimated efficiency 0.66.

Run :doc:`../../getting-started` and inspect the left torque channel:

.. code-block:: bash

   gz topic -e -t /model/payload_0/joint/front_left_wheel_joint/cmd_force

Compare torque with measured speed and the controller's duty logs. Changing the
feedforward full-speed calibration changes the requested duty estimate; it does
not change this motor physics model. See :doc:`../../control/feedforward`.

Source: `motor_sitl.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/drivers/motor/motor_sitl.cpp>`_.
