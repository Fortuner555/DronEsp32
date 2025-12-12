#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "manual_mode.h"
#include "motores.h"
#include <math.h>


// === SETUP INICIAL ===
void setup_manual_mode() {
  Serial.println("Setup manual mode completado.");
}

// === LOOP MANUAL CON SAMPLE-AND-HOLD DE REFERENCIA ===
void loop_manual_mode(float dt) {
  Serial.println("Loop manual mode.");

}
