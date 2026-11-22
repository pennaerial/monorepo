:doc-status: evolving
:status-reason: The filter runs in the computer-hosted simulation, but process-noise tuning, real hardware measurements, and some corrections remain unfinished.

Estimation
==========

An estimator answers “where are we, and how are we moving?” Sensors provide
partial, noisy evidence: the IMU predicts rapid changes, wheels constrain motion
along the ground, and camera position corrects accumulated drift.

The ``EKF`` class implements an ES-EKF. Start with :doc:`error-state-ekf` for
its nominal state, error state, and uncertainty model. Estimation is currently
independent of wheel speed control: the controller uses encoders directly.

.. mermaid::

   flowchart TB
       accTitle: EKF prediction and correction
       accDescr: IMU predicts the state. Wheel and camera data provide independent corrections. The result is estimated state and uncertainty.
       imu["IMU · timestamp, gyro, specific force"]
       wheel["Both wheel speeds + gyro"]
       camera["Camera · world XYZ position"]
       predict["Predict state and covariance"]
       wc["Correct body velocity"]
       cc["Correct world position"]
       state[("State + uncertainty · position, velocity, attitude, biases")]
       imu --> predict
       wheel --> wc
       camera --> cc
       predict --> wc --> cc --> state
       state -.->|"next fresh IMU sample"| predict
       classDef sensor fill:#e0f2fe,stroke:#0284c7,color:#0c4a6e
       classDef compute fill:#ede9fe,stroke:#7c3aed,color:#3b0764
       class imu,wheel,camera sensor
       class predict,wc,cc,state compute

Read :doc:`error-state-ekf` first for an explanation built from physical examples,
including why the filter uses a separate small rotation error and how it chooses
a correction. Then read :doc:`ekf` for equations and code mapping, and
:doc:`integration` for timing and which part of the program may change filter state.
The API reference is for exact function/type declarations.

.. toctree::
   :maxdepth: 1

   error-state-ekf
   ekf
   integration

Lookup: :doc:`../reference/estimator`.
