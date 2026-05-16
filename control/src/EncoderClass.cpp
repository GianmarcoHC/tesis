#include "EncoderClass.h"
#include "esp_log.h"

static const char *TAG = "ENCODER";

EncoderClass::EncoderClass(int pinA, int pinB, pcnt_unit_t unit)
    : pinA(pinA), pinB(pinB), unit(unit), count(0) {}

void EncoderClass::begin() {
    // --- Configuración del canal principal (A como pulso, B como control)
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = pinA;
    pcnt_config.ctrl_gpio_num = pinB;
    pcnt_config.unit = unit;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;

    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    // --- Filtro antirruido (ignora pulsos < 1 µs aprox)
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    // --- Inicialización del contador ---
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);

    ESP_LOGI(TAG, "Encoder inicializado (A=%d, B=%d, UNIT=%d)", pinA, pinB, unit);
}

int16_t EncoderClass::read_raw() {
    int16_t value = 0;
    esp_err_t err = pcnt_get_counter_value(unit, &value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo PCNT: %d", err);
    }
    return value;
}

float EncoderClass::read_degrees() {
    int16_t value = read_raw();
    float counts_per_rev = (float)(cpr * gear_reduction * edges_per_pulse);
    return (value * 360.0f) / counts_per_rev;
}

void EncoderClass::reset() {
    pcnt_counter_clear(unit);
}
