#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>   // aporta la clase Servo y setPeriodHertz()
#include <cstddef>

// Estos objetos se DEFINEN en variables.cpp
extern Servo mot1, mot2, mot3, mot4;

// Defaults de rango (ajusta si quieres)
#ifndef MOTOR_PWM_MIN_US
  #define MOTOR_PWM_MIN_US 1000
#endif
#ifndef MOTOR_PWM_MAX_US
  #define MOTOR_PWM_MAX_US 2000
#endif
#ifndef MOTOR_PWM_IDLE_US
  #define MOTOR_PWM_IDLE_US IDLE_PWM   // ya lo tienes en variables.cpp
#endif

// API similar a led.h
void init_motores_ctrl();
bool set_motor(uint8_t id, bool on);
bool encender_motor(uint8_t id);
bool apagar_motor(uint8_t id);
bool toggle_motor(uint8_t id);

bool set_motor_speed(uint8_t id, int us);
bool set_motor_on_with_speed(uint8_t id, int us);

void encender_todos_los_motores();
void apagar_todos_los_motores();
void set_motores_ids_onoff(const uint8_t* ids, size_t count, bool on);
void set_all_motors_speed(int us);

uint8_t motores_count();
bool motor_id_ok(uint8_t id);

// Compat con código existente
void apagarMotores();             // llama a apagar_todos_los_motores()
void encenderMotores(int speed);  // llama a set_all_motors_speed(speed)

// Tu control mezcla (ya lo tienes implementado en motores.cpp)
void applyControl(float tau_x, float tau_y, float tau_z);
void setupMotores(); 