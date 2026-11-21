Encoders
========

An encoder measures shaft rotation as counts (ticks). Differencing counts over
time estimates angular speed. The project uses **617 counts per output-shaft
revolution**, including **quadrature decoding**, which uses two offset electrical pulse signals
to count rotation and determine its direction; do not apply the gearbox ratio again
when interpreting that output-shaft speed.

For a count change ``delta_n`` over ``dt`` seconds:

.. math::

   \omega = \frac{2\pi\,\Delta n}{617\,\Delta t}

The common ``Encoder`` interface stores counts and angular velocity with separate
freshness counters. A new count is not necessarily a new velocity estimate.
Payload publishes counts, feeds fresh speed into :doc:`../../control/index`, and
pairs both sides for :doc:`../../estimation/integration`.

.. toctree::
   :maxdepth: 1

   sitl
   hardware
