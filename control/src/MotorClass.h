#pragma once

#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_err.h"

class MotorClass {
public:
    // Constructor: PWM + DIR
    MotorClass(gpio_num_t pwmPin, gpio_num_t dirPin,
               mcpwm_unit_t unit = MCPWM_UNIT_0,
               mcpwm_timer_t timer = MCPWM_TIMER_0,
               mcpwm_operator_t op = MCPWM_OPR_A);

    // Inicializa el módulo MCPWM
    void MotorInitialize();

    // pwmValue en rango [-100.0, 100.0] (% de duty)
    void setPWM(float pwmValue);

    // Apaga el motor
    void stop();

private:
    gpio_num_t pwmPin;
    gpio_num_t dirPin;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_operator_t op;
};