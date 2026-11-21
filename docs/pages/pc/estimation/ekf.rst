:doc-status: evolving
:status-reason: Process-noise densities and lever arms are currently zero; magnetometer/stationary corrections are placeholders and noise calibration is provisional.

ES-EKF Equations and Code
=========================

Start with :doc:`error-state-ekf` for the beginner explanation and worked example,
and the :doc:`IMU introduction <../sensors/imu/index>` for what the sensor measures.
This is an **error-state EKF**, although its C++ class is named ``EKF``.
The goal is a motion estimate that predicts between measurements and corrects
itself as independent evidence arrives. All equations here describe the current
``components/payload/ekf.cpp`` implementation.

State, uncertainty, and coordinates
-----------------------------------

The nominal state is world position ``p``, world velocity ``v``, body-to-world
quaternion ``q``, gyro bias ``b_g``, and accelerometer bias ``b_a``. The quaternion
is normalized and represents a 3D orientation, rather than three independent angles.

The 15-component error vector is ordered:

.. math::

   \delta x = [\delta p,\;\delta v,\;\delta\theta,\;\delta b_g,\;\delta b_a]^T

Each group has three entries. ``P`` (``covariance``) is 15-by-15: its diagonal
contains squared uncertainties; off-diagonal blocks describe correlations.
The orientation error has three small-angle components even though the nominal
quaternion has four coefficients.

``init(start_position)`` sets the known IMU position, zero velocity/biases, identity
orientation, and diagonal initial covariance. The initial standard deviations
are 0.1 m position, 0.2 m/s velocity, 0.1 rad attitude, 0.01 rad/s gyro bias, and
0.05 m/s² accelerometer bias. Starting in a different attitude requires a deliberate
initialization change, not an assumption that the quaternion will instantly recover.

Prediction: move the estimate forward
-------------------------------------

``base_adjustment`` subtracts estimated biases from raw body measurements. With
body-to-world rotation matrix ``R``, it obtains world acceleration:

.. math::

   \omega &= \omega_m-b_g \\
   a_W &= R(f_m-b_a) + [0,0,-9.81]^T \\
   p^+ &= p + v\Delta t + \tfrac12 a_W\Delta t^2 \\
   v^+ &= v + a_W\Delta t

The orientation right-multiplies a quaternion made from local rotation vector
``omega * dt``, then normalizes. Specific force includes support against gravity,
so an upright stationary reading produces approximately zero world acceleration.

The motion model is nonlinear because orientation changes how body force maps into
the world. **Linearizing** means approximating the effect of small changes near
the current estimate. The code puts these local relationships into matrix ``F``
and uses first-order transition ``Phi = I + F*dt``:

.. math::

   P^+ = \Phi P\Phi^T + G Q_c G^T\Delta t

``G`` maps gyro/accelerometer white noise and bias random walk into the error state.
**White noise** models rapidly varying, uncorrelated measurement errors; a
**random walk** models bias slowly wandering as small changes accumulate.
``Qc`` gives the continuous-time noise strengths and ``dt`` scales their contribution
to this prediction interval.
All four continuous-time process-noise standard deviations are currently **zero**.
That does not mean real sensors are noiseless; it is an unfinished calibration
choice. First-order discretization also assumes sufficiently small time steps;
there is no current upper-gap rejection in the timestamp path.

Wheel correction: constrain body velocity
-----------------------------------------

The forward wheel-based speed is ``radius * (left_rad_s + right_rad_s) / 2``.
The current wheel radius is 0.01611839 m. The model assumes no lateral slip, so its
two observed velocity components are forward speed and zero lateral speed.
It predicts axle velocity as:

.. math::

   v_{axle}^B = R^T v_W + (\omega_m-b_g)\times\ell_{axle}^B

The residual is observed minus predicted X/Y components. The lever arm ``ell``
accounts for the fact that a rotating offset point moves differently from the IMU
origin. It is currently zero. Both velocity measurement standard deviations are
0.02 m/s. Wheel slip violates the no-lateral-motion model; counts alone do not
prove that the body moved the expected distance.

Camera correction: anchor world position
----------------------------------------

The predicted camera position is ``p + R*camera_offset``. The residual subtracts
that prediction from the fresh world camera position. The Jacobian includes the
position block and an attitude block from the lever arm. With the current zero
offset, camera position has no direct lever-arm attitude term. Each axis uses
0.05 m measurement standard deviation.

The :doc:`simulated camera <../sensors/camera/sitl>` adds fixed bias as well as
white noise. The filter does not explicitly estimate that camera bias.

Correction: decide how much to trust the evidence
-------------------------------------------------

``compute_gain(H, R_z, residual)`` combines current uncertainty with measurement
uncertainty. ``H`` is the measurement Jacobian: how a small state error changes
the predicted measurement. ``R_z`` is measurement noise covariance, separate from
the rotation matrix ``R`` above.

.. math::

   S &= HPH^T + R_z \\
   K &= PH^TS^{-1} \\
   \delta x &= K(z-h(x))

The implementation uses **LDLT**, a matrix factorization used to solve a linear
system, rather than explicitly computing an inverse. Think of this as solving
for the unknown gain directly instead of constructing a separate inverse matrix.
The **innovation covariance** ``S`` is the uncertainty expected in the residual;
``K`` uses it to scale the residual into a state correction. It rejects non-finite values and invalid or nearly singular
innovation covariance. It injects position, velocity, and bias corrections, and
right-multiplies the attitude correction quaternion.

Covariance uses the Joseph form, then an attitude-error reset Jacobian and
symmetrization to reduce numerical asymmetry:

.. math::

   P_{corrected} = (I-KH)P(I-KH)^T + K R_z K^T

This protects numerical structure; it does not replace sensor validity checks or
outlier rejection. There is no current innovation-magnitude gate. Magnetometer
and stationary adjustments are TODOs and are not called by ``update()``.

Try tracing one correction
--------------------------

Run :doc:`../getting-started`, compare camera ground truth/noisy positions and
EKF position logs, and follow ``camera_adjustment -> compute_gain``. Notice that
reducing measurement covariance increases trust in that measurement, but cannot
remove a systematic measurement bias by itself. Before changing constants,
identify the measurement's units, frame, sample rate, and actual noise source.

Next: :doc:`integration`. Lookup: :doc:`../reference/estimator`.
Further learning: `Solà, Quaternion kinematics for the error-state Kalman filter
<https://arxiv.org/abs/1711.02508>`_ develops the rotation/error-state mathematics
in more depth; this project's sensor models and tuning remain implementation-specific.
