#include "pid.hpp"

float PID::update(float deltaT, float reading) {
    if (deltaT <= 0) {
        return 0;
    }

    float error = setPoint - reading;
    float p = kP * error;
    
    float d = 0;
    if (!secondReading) {
        secondReading = true;
        pastReading = reading;
    } else {
        float der = (pastReading - reading) / deltaT;
        d = kD * der;
        pastReading = reading;
    }

    i += kI * error * deltaT;

    // OMG it reads PID
    return p + i + d;
}

void PID::new_set_point(float nSetPoint) {
    setPoint = nSetPoint;
}