#ifndef LED_H
#define LED_H

#pragma once
#include <Arduino.h>

// Inicializa todos los LEDs (configura pines y apaga todos)
void init_leds();

// Encender / apagar / toggle de un LED por id (id empieza en 1)
bool set_led(uint8_t id, bool on);
bool encender_led(uint8_t id);
bool apagar_led(uint8_t id);
bool toggle_led(uint8_t id);

// Encender / apagar todos
void encender_todos_los_leds();
void apagar_todos_los_leds();

// Encender/apagar lista de IDs
void set_leds_ids_onoff(const uint8_t* ids, size_t count, bool on);

// Utilidad: cantidad total de LEDs configurados
uint8_t leds_count();


#endif
