:doc-status: evolving
:status-reason: The loop uses provisional calibration and holds the last duty indefinitely without a stale-feedback timeout.

PWMController: Feedforward Plus PI
==================================

``PWMController`` composes :doc:`feedforward` and :doc:`pid`. The class returns
a normalized request rather than
configuring GPIO waveforms itself. Hardware PWM belongs to
:doc:`../sensors/motors/hardware`.

For each new speed sample, ``command(dt, reading)``:

1. Computes a tentative PI output, including this sample's integral contribution.
2. Computes feedforward from the setpoint.
3. Checks whether their sum reaches/exceeds +1 while error is positive, or -1
   while error is negative. In those cases integration pushes further into saturation.
4. Recalculates feedback through the anti-windup path, restoring the preceding
   integral contribution.
5. Returns the sum of feedforward and feedback clamped to ``[-1,1]``.

If the error drives output back out of saturation, integration is permitted.
``change_set_point`` updates both subcontrollers so their targets stay aligned.
The active derivative gain is zero; this is PI plus feedforward.

Where it runs
-------------

``Payload::update_motor`` is called separately for left/right wheels. It commands
zero on the first velocity sample to seed elapsed time, computes a new duty only
on fresh velocity, and republishes the last duty between samples. The dt is
wall-clock elapsed time, while SITL velocity is computed using simulation time.
See :doc:`../payload` before interpreting pause/slow-simulation behavior.

Observe a complete loop
-----------------------

Run :doc:`../getting-started`; inspect encoder speed and motor torque channels.
Detailed setpoint/error/duty messages use ``ESP_LOGD`` and may require enabling
Debug log level for the Linux build through:

.. code-block:: bash

   idf.py -B build/linux -D SDKCONFIG=sdkconfig.linux menuconfig
   make linux
   GZ_WORLD=sae GZ_MODEL=payload_2 ./build/linux/payload_controller.elf

Choose the appropriate log verbosity in the IDF menu. A saturated request tells
you the controller cannot provide its desired effort; inspect target speed, load,
feedforward, sign, and measurement latency before increasing integral gain.
A simulated torque is not numerically equal to duty; the motor model converts it.

Lookup: :doc:`../reference/controllers`.
Source: `pwmcontroller.cpp <https://github.com/pennaerial/monorepo/blob/main/payload_controller/components/util/pwmcontroller.cpp>`_.
