Shared Helpers and Sample Ownership
===================================

The small helpers in ``components/util`` and ``components/sitl`` let drivers share
information without making estimation or control depend on a particular device.
Read :doc:`sensors/index` and :doc:`payload` first for the overall data flow.

A mutex protects a consistent snapshot
--------------------------------------

Gazebo may deliver a sensor callback while the application reads the previous
sample. Without
coordination, the application could read part of an old sample and part of a new one.

``StaticMutex`` wraps the appropriate lock: ``std::mutex`` for native Linux
threads, or a statically allocated FreeRTOS mutex on the physical board. ``lock()`` waits until
the caller can enter the protected section; ``unlock()`` allows another caller in.
The object cannot be copied or moved because each cache must retain its own lock.

``StaticMutexGuard`` locks in its constructor and unlocks in its destructor. This
is the C++ “resource acquisition is initialization” pattern, usually called RAII:
leaving the block, even through an early return, releases the lock automatically.

.. code-block:: cpp

   {
     util::StaticMutexGuard lock(mutex_);
     position_ = position;
     ++update_count_;
   } // unlocked here

Keep only the snapshot copy and counter change inside the lock. Do not run a
filter, wait for network delivery, or print lengthy debug output while holding it.
These locks protect driver caches, not the EKF or DDS client as a whole.

Freshness counters are per consumer
-----------------------------------

A driver increments a counter when it writes a new value. Each consumer passes
its own last-seen counter into ``get_latest_if_updated``. Equal counters mean
nothing new for that consumer; otherwise it receives the current value and its
cursor advances. The counter tells you that data changed, not how old it is.
It is not a timestamp or a queue of all readings. The current code also uses
unsigned counters, which can eventually wrap.

IMU/camera keep a sample counter; encoders keep separate count and velocity
counters. Payload keeps independent cursors for debug, estimator, and control.
This is why adding a debug read must not reuse the controller's cursor.

LogRateLimiter limits logs, not sensor processing
-------------------------------------------------

``LogRateLimiter::ready()`` returns true on its first call, then at most once per
20 ms using a monotonic steady clock. Each sensor/logging owner has its own
instance. Check it around logging, not around acquisition or a necessary control
update. The helper has no mutex and should not be shared between unrelated threads.
A slow application loop can log less often than 20 ms; the helper does not create
a task or schedule exact periodic output.

Side chooses the left/right physical drive
------------------------------------------

``payload_corner.hpp`` defines ``Side::LEFT`` and ``Side::RIGHT``. Each side has
one motor driving its pair of wheels. Motor/encoder factories use Side to select
separate static instances; it is not an array index with four corner values.
The IMU and Camera each have one shared instance instead.

Next: :doc:`simulation` for transport setup and message conversion.
Source: `utility helpers <https://github.com/pennaerial/monorepo/tree/main/payload_controller/components/util>`_.
