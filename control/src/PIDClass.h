#pragma once

class PIDClass {
private:
    float kp, ki, kd;
    float dt;
    float prevError;
    float integral;

public:
    PIDClass(float kp, float ki, float kd, float dt);
    float compute(float setpoint, float measured);
    void reset();
};