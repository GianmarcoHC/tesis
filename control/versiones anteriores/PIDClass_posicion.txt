#include "PIDClass.h"

PIDClass::PIDClass(float kp, float ki, float kd, float dt)
: kp(kp), ki(ki), kd(kd), dt(dt), prevError(0), integral(0) {}

float PIDClass::compute(float setpoint, float measured) {
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    float output = kp * error + ki * integral + kd * derivative;
    
    // Limitar a rango [-100, 100]
    if (output > 30.0f) output = 30.0f;
    if (output < -30.0f) output = -30.0f;

    return output;
}

void PIDClass::reset() {
    integral = 0;
    prevError = 0;
}