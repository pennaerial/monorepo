#pragma once

#include <string>
#include <vector>

// Records one lgpio call made during a test.
struct LgpioCall {
    std::string fn;
    int handle;
    int pin;
};

// Clear recorded calls between tests.
void mock_lgpio_reset();

// Inspect recorded calls.
const std::vector<LgpioCall>& mock_lgpio_calls();
