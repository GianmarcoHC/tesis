#include "MotorClass.h"

// Constructor
MotorClass::MotorClass(gpio_num_t pwmPin, gpio_num_t dirPin,
                       mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op)
    : pwmPin(pwmPin), dirPin(dirPin), unit(unit), timer(timer), op(op) {}

// Inicialización MCPWM
void MotorClass::MotorInitialize() {
    // Seleccionar automáticamente el canal correcto
    mcpwm_io_signals_t signal;

    if (timer == MCPWM_TIMER_0) signal = (op == MCPWM_OPR_A) ? MCPWM0A : MCPWM0B;
    else if (timer == MCPWM_TIMER_1) signal = (op == MCPWM_OPR_A) ? MCPWM1A : MCPWM1B;
    else signal = (op == MCPWM_OPR_A) ? MCPWM2A : MCPWM2B;

    // Configurar PWM output
    mcpwm_gpio_init(unit, signal, pwmPin);

    // Configurar pin de dirección
    gpio_set_direction(dirPin, GPIO_MODE_OUTPUT);
    gpio_set_level(dirPin, 0);

    // Configurar MCPWM
    mcpwm_config_t pwm_config = {
        .frequency = 20000,           // 20 kHz
        .cmpr_a = 0,                  // duty inicial
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };

    mcpwm_init(unit, timer, &pwm_config);
}

// Ajustar PWM (rango -100 a +100)
void MotorClass::setPWM(float pwmValue) {
    if (pwmValue > 100.0f) pwmValue = 100.0f;
    if (pwmValue < -100.0f) pwmValue = -100.0f;

    if (pwmValue > 0.0f) {
        gpio_set_level(dirPin, 1);
        mcpwm_set_duty(unit, timer, op, 100.0 - pwmValue);
    } else {
        gpio_set_level(dirPin, 0);
        mcpwm_set_duty(unit, timer, op, - pwmValue);
    }
    // Duty Cycle
    mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
}

// Apagar motor
void MotorClass::stop() {
    mcpwm_set_duty(unit, timer, op, 0);
    mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
    gpio_set_level(dirPin, 0);
}