#pragma once
#include "driver/pcnt.h"
#include "esp_err.h"

class EncoderClass {
private:
    int pinA;
    int pinB;
    pcnt_unit_t unit;
    int16_t count;

    // Parámetros del encoder
    int cpr = 13;              // Pulsos por revolución del encoder
    int gear_reduction = 75;   // Relación de reducción mecánica
    int edges_per_pulse = 2;   // Dos flancos por ciclo (A y B)

public:
    EncoderClass(int pinA, int pinB, pcnt_unit_t unit = PCNT_UNIT_0);

    void begin();          // Inicializa el PCNT
    int16_t read_raw();    // Devuelve el conteo crudo (en pulsos)
    float read_degrees();  // Devuelve la posición angular en grados
    void reset();          // Resetea el contador
};
