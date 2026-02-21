// Link-seam mock for lgpio.
// Compiled into the test target instead of the real lgpio library.
// Records every call so tests can assert on what was (or wasn't) invoked.

#include "mock_lgpio.hpp"
#include <lgpio.h>

static std::vector<LgpioCall> g_calls;

void mock_lgpio_reset()                            { g_calls.clear(); }
const std::vector<LgpioCall>& mock_lgpio_calls()   { return g_calls; }

// ---- lgpio stubs ----

int lgGpioWrite(int handle, int gpio, int /*value*/)
{
    g_calls.push_back({"lgGpioWrite", handle, gpio});
    return 0;
}

int lgTxPwm(int handle, int gpio,
            float /*freq*/, float /*duty*/,
            int /*offset*/, int /*cycles*/)
{
    g_calls.push_back({"lgTxPwm", handle, gpio});
    return 0;
}

int lgTxServo(int handle, int gpio,
              int /*pulseWidth*/, int /*freq*/,
              int /*offset*/, int /*cycles*/)
{
    g_calls.push_back({"lgTxServo", handle, gpio});
    return 0;
}
