Build Checks and Development Tools
==================================

The ``linux`` target runs on your computer for simulation; ``esp32s3`` produces
firmware for the board. **Static analysis** inspects source code without running
the robot. A **test assertion** checks whether an observed result matches an
expected one. Build success means code compiled, not that those behaviors were tested.

Start with the normal build commands in :doc:`getting-started` and the dependency
explanation in :doc:`build-system`. A compiler check, static analysis, and a
running simulation answer different questions; none alone verifies a physical robot.

What clang-tidy.sh does today
-----------------------------

From ``payload_controller/`` after preparing ESP-IDF and repository paths:

.. code-block:: bash

   ./scripts/clang-tidy.sh linux
   ./scripts/clang-tidy.sh esp32s3

**clang-tidy** reads C++ and compiler settings to find suspicious code. The script
creates a separate ``build/<target>-clang-tidy`` tree and separate SDKCONFIG so
normal GCC builds keep their own configuration. Linux uses ``clang-18`` and
``clang++-18``; ESP selects the installed ``esp-clang`` through ``IDF_TOOLCHAIN``.
It needs ``run-clang-tidy`` and the selected compiler installed first.

Why another compile database? GCC accepts flags that Clang may reject. Analyzing
the normal GCC command list with Clang can fail before it reaches your code.
The script instead configures matching compiler commands, then invokes
``run-clang-tidy`` using that database.

``.clang-tidy`` also limits header diagnostics to ``main/`` and ``src/``.
The current final filter matches ``main/`` and ``src/``. Most firmware code is
under ``components/``, so this script **does not currently cover most components**.
A successful run should not be reported as checking every driver or the EKF.
The docs describe this limit; changing the analysis filter is separate code work.

What the test files do today
----------------------------

``cmake/tests.cmake`` defines ``register_linux_test``. It parses an optional list
of libraries, creates an executable, links the current component plus Unity,
and registers a CTest command. Unity is a C/C++ assertion framework.

``components/sitl/test/CMakeLists.txt`` calls the helper for ``test_convert.cpp``.
Despite that filename, the current test only checks ``42 == 42``. It does not test
message conversion or estimator behavior. The top-level project also does not
currently connect a complete test-discovery path. There is no supported
``make test`` command that proves payload correctness.

Useful checks while that work is unfinished
-------------------------------------------

1. Build both ``make linux`` and ``make esp32s3`` after backend or dependency changes.
2. Run the relevant simulation path and compare input/output units and signs.
3. For conversion changes, design checks for timestamps, missing fields, fixed
   array sizes, and frame-name bounds; the placeholder test is not evidence for these.
4. For control/estimation changes, compare repeatable sensor inputs with expected
   output and explicitly test timing resets or missing samples.
5. Keep hardware behavior marked unfinished until its producer and bench test exist.

Use :doc:`code-map` to find the guide for each source/build file.
