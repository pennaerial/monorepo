:doc-priority: key
:key-reason: Start here for estimation work: understand nominal state, local error, and prediction/correction before the implementation.
:doc-status: evolving
:status-reason: This explains the current ES-EKF design; process-noise tuning, hardware integration, and some planned corrections remain unfinished.

Understanding the Error-State EKF
=================================

This page builds up the idea without assuming you have studied estimation.
Read the :doc:`system overview <../overview>` first. The next page,
:doc:`ekf`, maps the same ideas to the matrices and functions in this project.
The code class is named ``EKF``, but its design is an **error-state extended
Kalman filter**, abbreviated **ES-EKF** or **ESKF**.

Why the controller needs an estimate
------------------------------------

Imagine asking a payload to report where it is after rolling around a room.
No single current sensor gives a complete, exact answer:

* The IMU tells us about rotation and acceleration-like forces, not position.
* Wheel encoders measure how far the shafts turned. A slipping wheel can turn
  without moving the body as far as expected.
* A camera-position source can anchor location, but its readings can be noisy,
  delayed, or unavailable. This project's camera is currently a simulated stand-in.

An **estimate** is our best working answer from that imperfect evidence. It is
not ground truth. The filter also keeps a model of how uncertain that answer is.
A small persistent sensor offset, called **bias**, matters: even while stationary,
a gyro bias can make integrated orientation slowly turn, and an accelerometer
bias can make estimated velocity and position drift.

What “extended Kalman filter” means
-----------------------------------

A Kalman filter alternates two jobs: **predict** what should happen next, then
**correct** that prediction when a measurement supplies new evidence. The amount
of correction depends on the uncertainty assigned to the prediction and measurement.

“Extended” means the motion/measurement relationships are not all straight-line
relationships. For example, which world direction an accelerometer points depends
on the payload's orientation. The filter uses a local approximation around its
current estimate, rather than pretending one linear model works for every pose.
This makes a good starting estimate, correct units, and suitable time steps important.

Two descriptions of the same situation
--------------------------------------

The **nominal state** is the main answer: current estimated position, velocity,
orientation, gyro bias, and accelerometer bias. “Nominal” here means the reference
estimate about which we describe errors; it does not mean perfect or ideal.

The **error state** describes a small correction to that answer. If nominal X is
2.0 m and a new correction says +0.1 m, the updated nominal X becomes 2.1 m.
We do not keep adding two competing permanent position estimates. After applying
the correction, the corrected state becomes the reference for the next small error.

The error state is a mathematical estimate of the difference from truth. We do
not know the true error directly. New measurements give evidence about it, and
**covariance** records the filter's uncertainty about that error.

.. list-table:: State groups in this implementation
   :header-rows: 1
   :widths: 24 38 38

   * - Quantity
     - Main estimate
     - Small error used for correction
   * - Position
     - IMU's world X/Y/Z in metres
     - Three position corrections
   * - Velocity
     - IMU's world X/Y/Z speed in m/s
     - Three velocity corrections
   * - Orientation
     - Body-to-world unit quaternion (four coefficients)
     - Three local rotation corrections in radians
   * - Gyro bias
     - Three estimated offsets in rad/s
     - Three bias corrections
   * - Accelerometer bias
     - Three estimated offsets in m/s²
     - Three bias corrections

There are **15 error values**, grouped in that order. The main state has 16
stored coefficients because the quaternion uses four coefficients to describe
three independent rotation degrees of freedom (rotation around three axes).

Why orientation uses a small rotation error
-------------------------------------------

A quaternion is a compact way to describe a full 3D orientation without the
singularities of a three-angle representation. Its four coefficients obey a
unit-length constraint. Adding four arbitrary correction numbers would not, by
itself, respect that constraint or describe the intended physical rotation.

The ES-EKF instead estimates a small three-number rotation vector. Its direction
is the rotation axis; its length is the angle in radians. The code turns that
vector into a rotation quaternion, multiplies it into the main orientation, and
normalizes the result. In this implementation the correction multiplies on the
**right**, so it is expressed in local/body rotation coordinates. Swapping the
order would change the meaning and require different equations.

This is the “error-state” distinction: propagate the full orientation, but estimate
small local errors with simpler linear tools. It does not mean that every error
in every situation is guaranteed to remain small.

Step 1: predict with the IMU
----------------------------

A new IMU sample has a sensor timestamp, gyro rotation rate, and accelerometer
specific force. The difference from the previous accepted timestamp gives elapsed
time ``dt``. The first sample only establishes time; it cannot tell us how long
the unknown interval before startup lasted.

``base_adjustment`` first subtracts estimated biases. It rotates the corrected
specific force from body axes into world axes, then adds downward gravity.
At rest and upright, specific force is about +9.81 m/s² upward; adding -9.81 m/s²
world gravity leaves roughly zero world acceleration. A tilted body needs the
rotation first: subtracting 9.81 from an arbitrary body Z axis would be wrong.

It then advances position and velocity using elapsed time, and advances orientation
using gyro rate times elapsed time. For a simple straight-line example with correct
orientation, zero starting speed, and 1 m/s² world acceleration for 0.1 s, the
prediction adds 0.1 m/s to speed and 0.005 m to position. These are basic motion
equations, before any measurement correction.

Prediction also moves uncertainty forward. An uncertain tilt can create uncertain
world acceleration; that can create uncertain speed and then position. The filter
tracks these relationships, not just a separate confidence score on each number.

Step 2: say how uncertain the answer is
---------------------------------------

The covariance matrix ``P`` is a 15-by-15 table for the error state. A matrix is
just a rectangular table of numbers used for calculations. Its diagonal holds
**variances**: if position standard deviation is 0.1 m, its variance is
``0.1 * 0.1 = 0.01 m²``. Off-diagonal entries record relationships between errors.

For example, if earlier acceleration uncertainty made both position and velocity
uncertain in related ways, a position measurement can also change the velocity
estimate. The sensor did not directly measure velocity; the relationship stored
in covariance allows that indirect correction.

There are two distinct noise models:

* **Process noise** covers uncertainty added as the motion model predicts forward,
  such as noisy IMU readings and slowly changing biases.
* **Measurement noise** covers uncertainty assigned to a correcting observation,
  such as a wheel speed or camera position.

In the current code, the four process-noise densities are **zero**. That is an
unfinished tuning choice, not a claim that real IMUs have no noise. Measurement
noise values are also assumptions to validate. A covariance that looks small
can mean the filter is overconfident, not that its answer is accurate.

Step 3: compare a measurement with its prediction
-------------------------------------------------

The **measurement model** predicts what a sensor should report if the current
state were correct. The **residual**, also called an **innovation**, is:

.. math::

   \text{residual} = \text{observed value} - \text{predicted sensor value}

These values must have the same units and refer to compatible frames/locations.
A camera observes its own location, not automatically the IMU's. If it is mounted
away from the IMU, the predicted camera position includes that rotated offset.
That offset is called a **lever arm**. It is currently configured as zero here.

Wheel measurements constrain forward body speed from the average of both wheel
rates times wheel radius, and assume zero sideways axle speed. This is a
**no-sideways-slip assumption**, not a sideways speed sensor. Slipping or lifting
the wheels can violate it. A gyro/lever-arm term accounts for axle motion caused
by rotation when the axle is offset from the IMU; that offset is also currently zero.

Step 4: choose and apply a correction
-------------------------------------

The **Kalman gain** tells the filter how much of the residual to use for each
state correction. In the full filter it is a matrix because one observation can
change several related state quantities.

A one-dimensional position example makes the weighting easier to see. Suppose:

* Predicted position is 2.0 m, with variance ``P = 0.04 m²``.
* A position sensor reads 2.1 m, with noise variance ``R = 0.01 m²``.
* This simplified sensor measures that position directly, with no offsets or
  correlations to other state quantities.

The residual is +0.1 m. The scalar gain is ``P/(P+R) = 0.8``. Apply 80% of the
residual: the correction is +0.08 m and the new position is **2.08 m**.
If the sensor were much noisier, its gain would be smaller. This is an illustration
of the correction step, not a set of recommended constants for the payload.

For the real 15-value error state, the camera/wheel adjustment functions build
a local measurement-sensitivity matrix. ``compute_gain`` uses that matrix to
combine state and sensor uncertainty, solve for the gain, and obtain a correction
vector. It adds position, velocity, and bias corrections
to their main estimates. Orientation uses the small quaternion rotation described
above. The state is not simply replaced with the latest sensor reading.

Step 5: reset the error around the corrected state
--------------------------------------------------

Once the correction is applied, the updated main state becomes the new reference.
The estimated local error is conceptually reset to zero. **Zero error estimate
does not mean zero uncertainty**: ``P`` still describes how wrong we might be.

This code does not keep a persistent 15-value mean-error member. It computes a
local correction ``dx``, applies it, and retains the corrected main state and
covariance. It updates covariance using the Joseph form, a calculation designed
to preserve its uncertainty structure in finite-precision arithmetic. It then
uses an attitude reset Jacobian to express orientation uncertainty around the
new reference, and makes the matrix symmetric to reduce numerical round-off drift.
The detailed page shows those calculations.

How this fits the actual program
--------------------------------

``Payload::update_estimator`` collects fresh cached measurements. It calls
``EKF::update`` once with whichever inputs are available. That function predicts
from the IMU first, then applies a wheel correction, then a camera correction.
A camera correction can run without waiting for a fresh IMU sample in that pass.
The code's latest-value caches do not preserve or time-align every sensor reading.

The filter estimates IMU position and motion. The current motor speed controller
still reads encoder velocity directly; it does not use the EKF's position to
choose speed targets. Magnetometer and stationary corrections are TODOs, not
active steps hidden inside the equations. The filter has numerical validity
checks but no current innovation-size outlier gate, so a finite but incorrect
measurement can still pull the estimate away.

Next, read :doc:`ekf` for the code/equation mapping and :doc:`integration` for
sample timing and ownership. Use :doc:`../glossary` for quick term lookups.
For a deeper derivation, see `Solà's error-state and quaternion tutorial
<https://arxiv.org/abs/1711.02508>`_; the sensor models and unfinished tuning above
are specific to this repository.
