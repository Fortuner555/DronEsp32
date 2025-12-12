#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "motores.h"
#include "variables.h"
#include "mpu.h"
#include <Wire.h>
#include "piloto_mode.h"
#include "manual_mode.h"
#include <esp_task_wdt.h>
#include "soc/rtc_cntl_reg.h"
#include "config.h"
#include "UDP.hpp"
#include "ESP32_Utils.hpp"
#include "ESP32_Utils_UDP.hpp"
#include <cstddef>
#include "led.h"

SemaphoreHandle_t sensorMutex = NULL;

// ================= VARIABLES =================
volatile bool ledState = false;
volatile bool motorState = false;
volatile bool modoCambiado = false;

Preferences preferences;
TaskHandle_t TaskControl;
TaskHandle_t TaskComunicacion;
void changeMode(int newMode);

// ===== Helpers JSON =====
static inline int clamp_speed_int(int us) {
  return constrain(us, MOTOR_PWM_MIN_US, MOTOR_PWM_MAX_US);
}

static bool read_i(JsonVariant v, int &out) {
  if (v.is<int>())         { out = v.as<int>(); return true; }
  if (v.is<const char*>()) { out = atoi(v.as<const char*>()); return true; }
  return false;
}

// Acepta: 0/1/2, "0"/"1"/"2", "pilot"/"piloto", "manual", "idle"/"espera"
static bool parse_mode(JsonVariant v, int &out) {
  if (read_i(v, out)) return true;
  if (v.is<const char*>()) {
    String s = v.as<const char*>(); s.toLowerCase();
    if (s == "pilot" || s == "piloto") { out = 0; return true; }
    if (s == "manual")                 { out = 2; return true; }
    if (s == "idle"   || s == "espera"){ out = 1; return true; }
  }
  return false;
}

static bool read_u8(JsonVariant v, uint8_t &out) {
  if (v.is<int>())         { out = (uint8_t)v.as<int>(); return true; }
  if (v.is<const char*>()) { out = (uint8_t)atoi(v.as<const char*>()); return true; }
  return false;
}

static const bool REQUIRE_IDLE_FOR_EXTERNAL = false;
static inline bool can_accept_external() {
  return !REQUIRE_IDLE_FOR_EXTERNAL || (modoActual == 1);
}

static void apply_led(uint8_t id, bool state) {
  if (state) encender_led(id);
  else       apagar_led(id);
  Serial.printf("LED %u -> %s\n", id, state ? "ON" : "OFF");
}

static void apply_leds(JsonArray ids, bool state) {
  for (JsonVariant v : ids) {
    uint8_t id = v.as<uint8_t>();
    apply_led(id, state);
  }
}

static void apply_motor(uint8_t id, bool state) {
  if (!motor_id_ok(id)) {
    Serial.printf("⚠️ Motor id inválido: %u\n", id);
    return;
  }
  if (state) encender_motor(id);
  else       apagar_motor(id);
  motorState = state;
  Serial.printf("MOTOR %u -> %s\n", id, state ? "ON" : "OFF");
}

static void apply_motors_all(bool state) {
  if (state) encender_todos_los_motores();
  else       apagarMotores();
  motorState = state;
  Serial.printf("MOTORES -> %s\n", state ? "ON" : "OFF");
}

static void apply_motor_speed(uint8_t id, int us) {
  if (!motor_id_ok(id)) {
    Serial.printf("⚠️ Motor id inválido: %u\n", id);
    return;
  }
  set_motor_on_with_speed(id, us);  // ON directo a 'us'
  motorState = true;
  Serial.printf("MOTOR %u -> %d us\n", id, us);
}

static void apply_motors_all_speed(int us) {
  set_all_motors_speed(us);
  motorState = (us > 1000);
  Serial.printf("MOTORES ALL -> %d us\n", us);
}

static void send_ack(const char* request_id, bool ok, const String& info) {
  JsonDocument ack;
  ack["type"] = "ack";
  if (request_id) ack["request_id"] = request_id;
  ack["ok"]   = ok;
  ack["info"] = info;

  String out;
  serializeJson(ack, out);
  // Responde al remitente del último paquete (PC)
  SendUDP_PacketToLastPeer(out.c_str(), out.length());
}

void processMessage(const char *message) {
  JsonDocument doc;
  DeserializationError derr = deserializeJson(doc, message);
  if (derr) { Serial.println("❌ JSON inválido"); send_ack(nullptr,false,"json_parse_error"); return; }

  const char* req_id = nullptr;
  if (doc["request_id"].is<const char*>()) req_id = doc["request_id"].as<const char*>();
  else if (doc["payload"]["request_id"].is<const char*>()) req_id = doc["payload"]["request_id"].as<const char*>();

  bool handled = false;
  String info  = "";

  if (doc["type"].is<const char*>() && strcmp(doc["type"].as<const char*>(), "command") == 0) {

    JsonObject payload = doc["payload"].is<JsonObject>() ? doc["payload"].as<JsonObject>() : JsonObject();
    // soporta casos heredados con "payload" anidado
    JsonObject cmd = (payload["payload"].is<JsonObject>()) ? payload["payload"].as<JsonObject>() : payload;

    // ===== MODE (aceptar int o string) =====
    if (cmd["mode"].is<int>() || cmd["mode"].is<const char*>()) {
      int newMode = -1;
      if (!parse_mode(cmd["mode"], newMode)) {
        send_ack(req_id, false, "unknown_mode");
        return;
      }
      changeMode(newMode);
      handled = true; info = String("mode->") + newMode;
    }

    // ===== LED (igual que ya tienes) =====
    if (cmd["led"].is<bool>()) {
      bool st = cmd["led"].as<bool>();
      for (uint8_t i = 1; i <= leds_count(); ++i) apply_led(i, st);
      handled = true; info = "led all";
    }
    if (cmd["led"].is<JsonObject>()) {
      JsonObject obj = cmd["led"].as<JsonObject>();
      uint8_t id;
      if (read_u8(obj["id"], id) && obj["state"].is<bool>()) {
        apply_led(id, obj["state"].as<bool>());
        handled = true; info = "led one";
      }
    }
    if (cmd["leds"].is<JsonObject>()) {
      JsonObject obj = cmd["leds"].as<JsonObject>();
      if (obj["ids"].is<JsonArray>() && obj["state"].is<bool>()) {
        apply_leds(obj["ids"].as<JsonArray>(), obj["state"].as<bool>());
        handled = true; info = "leds many";
      }
    }

    // ===== MOTORS: uno =====
    if (cmd["motor"].is<JsonObject>()) {
      if (!can_accept_external()) { send_ack(req_id,false,"busy_mode"); return; }

      JsonObject obj = cmd["motor"].as<JsonObject>();
      uint8_t id;
      if (!read_u8(obj["id"], id)) { send_ack(req_id,false,"bad_motor_id"); return; }

      if (obj["state"].is<bool>()) {
        apply_motor(id, obj["state"].as<bool>());
        handled = true; info = "motor one on/off";
      } else if (obj["speed"].is<int>() || obj["speed"].is<const char*>()) {
        int us;
        if (!read_i(obj["speed"], us)) { send_ack(req_id,false,"bad_speed"); return; }
        apply_motor_speed(id, clamp_speed_int(us));
        handled = true; info = "motor one speed";
      }
    }

    // ===== MOTORS: varios / todos =====
    if (cmd["motors"].is<JsonObject>()) {
      if (!can_accept_external()) { send_ack(req_id,false,"busy_mode"); return; }

      JsonObject obj = cmd["motors"].as<JsonObject>();

      // ids + state
      if (obj["ids"].is<JsonArray>() && obj["state"].is<bool>()) {
        bool st = obj["state"].as<bool>();
        for (JsonVariant v : obj["ids"].as<JsonArray>()) {
          uint8_t id;
          if (read_u8(v, id)) apply_motor(id, st);
        }
        handled = true; info = "motors many on/off";
      }

      // ids + speed
      if (obj["ids"].is<JsonArray>() && (obj["speed"].is<int>() || obj["speed"].is<const char*>())) {
        int us; if (!read_i(obj["speed"], us)) { send_ack(req_id,false,"bad_speed"); return; }
        us = clamp_speed_int(us);
        for (JsonVariant v : obj["ids"].as<JsonArray>()) {
          uint8_t id;
          if (read_u8(v, id)) apply_motor_speed(id, us);
        }
        handled = true; info = "motors many speed";
      }

      // todos: state
      if (obj["state"].is<bool>() && !obj["ids"].is<JsonArray>()) {
        apply_motors_all(obj["state"].as<bool>());
        handled = true; info = "motors all (object)";
      }

      // todos: speed
      if ((obj["speed"].is<int>() || obj["speed"].is<const char*>()) && !obj["ids"].is<JsonArray>()) {
        int us; if (!read_i(obj["speed"], us)) { send_ack(req_id,false,"bad_speed"); return; }
        apply_motors_all_speed(clamp_speed_int(us));
        handled = true; info = "motors all speed";
      }
    }

    // legacy: {"motors": true/false}
    if (cmd["motors"].is<bool>()) {
      if (!can_accept_external()) { send_ack(req_id,false,"busy_mode"); return; }
      apply_motors_all(cmd["motors"].as<bool>());
      handled = true; info = "motors all (bool)";
    }
  }

  if (!handled && (doc["mode"].is<int>() || doc["mode"].is<const char*>())) {
    int newMode = -1;
    if (parse_mode(doc["mode"], newMode)) {
      changeMode(newMode);
      handled = true; info = String("mode->") + newMode;
    }
  }

  // get_mode
  if (!handled && doc["type"].is<const char*>() &&
      strcmp(doc["type"].as<const char*>(), "get_mode") == 0) {
    JsonDocument rep;
    rep["type"]  = "mode";
    rep["value"] = modoActual;
    String out; serializeJson(rep, out);
    SendUDP_PacketToLastPeer(out.c_str(), out.length());
    handled = true; info = "mode_sent";
  }

  if (!handled) info = "unknown_command_or_schema";
  send_ack(req_id, handled, info);
}


void TaskControlCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    int lastMode = -1;
    for (;;)
    {
        esp_task_wdt_reset();
        if (modoActual != lastMode)
        {
            lastMode = modoActual;
            switch (modoActual)
            {
            case 0:                         // Modo piloto
                digitalWrite(pinLed, HIGH); // Indicador de inicio
                Serial.println("Modo piloto activado");
                setup_pilote_mode();
                digitalWrite(pinLed, LOW); // Indicador de inicio
                break;
            case 2:                         // Modo manual
                digitalWrite(pinLed, HIGH); // Indicador de inicio
                Serial.println("Modo manual activado");
                setup_manual_mode();
                digitalWrite(pinLed, LOW); // Indicador de inicio
                break;
            case 1: // Modo espera
                Serial.println("Modo espera activado");
                apagarMotores();
                break;
            default:
                break;
            }
        }
        // Lectura de sensores SIEMPRE ACTIVA a 25Hz independiente del modo
        {
            static uint32_t sensor_last_time = 0;
            float sensor_dt = (micros() - sensor_last_time) / 1e6;
            if (sensor_dt >= 0.001)
            {
                if (xSemaphoreTake(sensorMutex, 0) == pdTRUE)
                {
                    gyro_signals();
                    xSemaphoreGive(sensorMutex);
                }
                sensor_last_time = micros();
            }
        }

        // Ejecutar loop solo si el modo es piloto o manual
        if (modoActual == 0 || modoActual == 2)
        {
            static uint32_t control_last_time = 0;
            float control_dt = (micros() - control_last_time) / 1e6;
            if (control_dt >= 0.001)
            {
                if (xSemaphoreTake(sensorMutex, 0) == pdTRUE)
                {
                    if (modoActual == 0)
                        loop_pilote_mode(control_dt);
                    else if (modoActual == 2)
                        loop_manual_mode(control_dt);
                    xSemaphoreGive(sensorMutex);
                }
                control_last_time = micros();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void TaskComunicacionCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    for (;;)
    {
        esp_task_wdt_reset();
        GetUDP_Packet(false);
        SendTelemetry();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void changeMode(int newMode)
{
    if (newMode != modoActual)
    {
        modoActual = newMode;
        modoCambiado = true;
        preferences.putInt("modo", modoActual);
        Serial.printf("Modo cambiado a: %d\n", modoActual);
        if (modoActual == 0)
        {
            Serial.println("Modo piloto activado");
        }
        else if (modoActual == 2)
        {
            Serial.println("Modo manual activado");
        }
        else if (modoActual == 1)
        {
            Serial.println("Modo espera activado");
        }
    }
}

void setup()
{
    pinMode(pinLed, OUTPUT);
    digitalWrite(pinLed, HIGH); // Indicador de inicio
    preferences.begin("dronData", false);
    modoActual = 1;                         // Siempre iniciar en modo espera
    preferences.putInt("modo", modoActual); // Guardar modo espera en la memoria
    ledState = preferences.getBool("ledState", false);
    digitalWrite(pinLed, HIGH); // Indicador de inicio
    WiFi.setSleep(false);

    ConnectWiFi_STA(true);
    setupMPU();
    ConnectUDP();
    digitalWrite(pinLed, HIGH); // Indicador de inicio
    btStop();
    digitalWrite(pinLed, HIGH);
    init_leds();
    digitalWrite(pinLed, HIGH);
    setupMotores();
    digitalWrite(pinLed, HIGH);
    esp_task_wdt_init(30, true);
    udpMutex = xSemaphoreCreateMutex();
    UDP.begin(localPort);
    sensorMutex = xSemaphoreCreateMutex();
    digitalWrite(pinLed, HIGH);
    xTaskCreatePinnedToCore(
        TaskControlCode,
        "TaskControl",
        10000,
        NULL,
        2, // Prioridad más alta para el control
        &TaskControl,
        0); // Núcleo 0 para control en tiempo real

    xTaskCreatePinnedToCore(
        TaskComunicacionCode,
        "TaskComunicacion",
        15000,
        NULL,
        1, // Prioridad menor para comunicación
        &TaskComunicacion,
        1); // Núcleo 1 para comunicación

    digitalWrite(pinLed, LOW);
    Serial.println("✅ Sistema inicializado correctamente");
}

void loop()
{
}