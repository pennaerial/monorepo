:doc-status: evolving
:status-reason: Fixed speed targets and provisional gains are used; hardware feedback integration and stale-feedback handling remain unfinished.

Motor Control
=============

A controller answers “how hard should the motor drive to reach the desired speed?”
A **setpoint** is the requested value: here, wheel/output-shaft speed in
**rad/s** (radians per second; one full turn is ``2*pi`` radians). **Duty** is
a unitless effort request from -1 to +1, not a speed. **PI** means proportional
plus integral feedback; **PID** adds a derivative term. **PWM** is the electrical
switching method used by the physical motor driver. **Feedforward** estimates
needed effort from the requested speed before a measured error is available.

Encoders provide measured speed; PWMController returns normalized signed duty. The motor backend turns that
into simulated torque or hardware bridge signals.

.. mermaid::

   flowchart TB
       accTitle: Wheel speed feedback loop
       accDescr: Feedforward estimates duty from target speed. PI uses target minus encoder speed. Their sum is clamped and applied to the motor, whose speed is measured again.
       target(["Desired wheel speed · rad/s"])
       ff["Feedforward · expected effort"]
       error["Error = desired − measured speed"]
       pi["PI · correct remaining error"]
       sum(("+"))
       clamp["Clamp duty · −1 to +1"]
       motor["Motor and wheel"]
       encoder["Encoder · measured rad/s"]
       target --> ff --> sum
       target --> error --> pi --> sum
       sum --> clamp --> motor --> encoder
       encoder --> error
       clamp -.->|"saturation test controls integration"| pi
       classDef sensor fill:#e0f2fe,stroke:#0284c7,color:#0c4a6e
       classDef compute fill:#ede9fe,stroke:#7c3aed,color:#3b0764
       classDef actuator fill:#ffedd5,stroke:#ea580c,color:#7c2d12
       class encoder sensor
       class ff,error,pi,sum,clamp compute
       class motor actuator

Feedforward predicts the effort required. Feedback corrects the remaining error.
The current PWMController uses **PI**, with derivative gain zero, even though its
underlying reusable class is named PID. Estimated EKF position is not an input to
this loop. See :doc:`../payload` for scheduling and sample freshness.

.. toctree::
   :maxdepth: 1

   feedforward
   pid
   pwmcontroller

Lookup: :doc:`../reference/controllers`.
