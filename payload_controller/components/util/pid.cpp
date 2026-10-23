#include "pid.hpp"

#include <cmath>
#include <algorithm>

float PID::update(float deltaT, float reading, bool antiWindup) {
    if (deltaT <= 0) {
        return 0;
    }

    float error = setPoint - reading;
    float p = kP * error;
    float d = 0;

    // When antiWindup is true, we are rerunning PID loop again so we want to avoid making same changes again
    if (!antiWindup) {
        if (!secondReading) {
            secondReading = true;
            pastReading = reading;
        } else {
            float der = (pastReading - reading) / deltaT;
            d = kD * der;
            pastReading = reading;
            pastD = d;
        }
        pastI = i;
        i += kI * error * deltaT;
        i = std::clamp(i, -integralMax, integralMax);
    } else {
        i = pastI;
        d = pastD;
    }

    // OMG it reads PID
    return p + i + d;
}

void PID::new_set_point(float nSetPoint) {
    constexpr float stopThreshold = 0.001f;

    bool wasMoving = std::abs(setPoint) >= stopThreshold;
    bool stopping = std::abs(nSetPoint) < stopThreshold;

    // Reset integral term when stopping
    if (wasMoving && stopping) {
        i = 0.0f;
    }

    setPoint = nSetPoint;
}