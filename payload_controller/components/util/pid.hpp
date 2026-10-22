#pragma once

class PID {

public:
    PID(float kP, float kD, float kI, float setPoint) : setPoint(setPoint), kP(kP),  kD(kD), kI(kI) {}

    float update(float deltaT, float reading);
    void new_set_point(float nSetPoint);

private:
    float setPoint;

    float kP;
    float kD;
    float kI;

    bool secondReading = false;
    float pastReading = 0;

    float i = 0;

};