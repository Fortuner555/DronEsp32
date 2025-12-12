#include "led.h"
#include <cstddef>

static const uint8_t LED_PINS[] = {
  2, 4, 5  // id=1->GPIO2, id=2->GPIO4, id=3->GPIO5 (ajústalo a tu hardware)
};
static const size_t LED_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

// Si tus LEDs son activos en bajo (se encienden con LOW), ponlo en false
static const bool LED_ACTIVE_HIGH = true;

static bool LED_STATE[sizeof(LED_PINS) / sizeof(LED_PINS[0])] = {false};

static inline bool led_id_ok(uint8_t n)            { return n >= 1 && n <= LED_COUNT; }
static inline uint8_t led_pin_from_id(uint8_t n)   { return LED_PINS[n - 1]; }
static inline int to_logic(bool on) {
  return LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
}

void init_leds() {
  for (size_t i = 0; i < LED_COUNT; ++i) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], to_logic(false)); // arrancar apagados
    LED_STATE[i] = false;
  }
}

bool set_led(uint8_t n, bool on) {
  if (!led_id_ok(n)) {
    Serial.printf("⚠️ LED id inválido: %u\n", n);
    return false;
  }
  const uint8_t pin = led_pin_from_id(n);
  digitalWrite(pin, to_logic(on));
  LED_STATE[n - 1] = on;
  Serial.printf("LED %u -> %s\n", n, on ? "ON" : "OFF");
  return true;
}

bool encender_led(uint8_t n) { return set_led(n, true); }
bool apagar_led(uint8_t n)   { return set_led(n, false); }

bool toggle_led(uint8_t n) {
  if (!led_id_ok(n)) {
    Serial.printf("⚠️ LED id inválido: %u\n", n);
    return false;
  }
  return set_led(n, !LED_STATE[n - 1]);
}

void encender_todos_los_leds() {
  for (uint8_t i = 1; i <= LED_COUNT; ++i) set_led(i, true);
}

void apagar_todos_los_leds() {
  for (uint8_t i = 1; i <= LED_COUNT; ++i) set_led(i, false);
}

void set_leds_ids_onoff(const uint8_t* ids, size_t count, bool on) {
  for (size_t k = 0; k < count; ++k) set_led(ids[k], on);
}

uint8_t leds_count() {
  return static_cast<uint8_t>(LED_COUNT);
}
