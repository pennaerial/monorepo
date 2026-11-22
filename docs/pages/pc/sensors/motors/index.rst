Motors
======

A motor is an actuator: it turns electrical input into torque. Payload does not
send a desired speed directly to the motor driver. The :doc:`../../control/index`
loop turns desired and measured speed into a signed duty request, then
``Motor::publish(double pwm)`` sends that request to the selected backend.

``Side::LEFT`` and ``Side::RIGHT`` select separate instances. Both
backends accept normalized duty in ``[-1,1]``; they implement different physical
models. SITL computes torque from voltage and speed, while hardware generates
H-bridge input waveforms. An encoder closes the speed-feedback loop.

.. toctree::
   :maxdepth: 1

   sitl
   hardware
