#include "motores.h"
#include "variables.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>

// ====== Tabla de punteros a tus motores ======
Servo* const MOTOR_PTRS[] = {
  &mot1, &mot2, &mot3, &mot4
  // añade aquí: &mot5, &mot6, ...
};
static const uint8_t MOTOR_COUNT = sizeof(MOTOR_PTRS) / sizeof(MOTOR_PTRS[0]);
static bool MOTOR_STATE[sizeof(MOTOR_PTRS) / sizeof(MOTOR_PTRS[0])] = {false}; // false=OFF

// ====== Helpers internos ======
static inline int clamp_us(int us) {
  if (us < MOTOR_PWM_MIN_US) return MOTOR_PWM_MIN_US;
  if (us > MOTOR_PWM_MAX_US) return MOTOR_PWM_MAX_US;
  return us;
}

bool motor_id_ok(uint8_t id) { return id >= 1 && id <= MOTOR_COUNT; }
static inline Servo* motor_ptr(uint8_t id) { return MOTOR_PTRS[id - 1]; }

uint8_t motores_count() { return MOTOR_COUNT; }

// ====== Operaciones principales ======
static void motor_write_us(uint8_t id, int us) {
  if (!motor_id_ok(id)) return;
  if (auto s = motor_ptr(id)) s->writeMicroseconds(clamp_us(us));
}

bool set_motor(uint8_t id, bool on) {
  if (on) {
    motor_write_us(id, MOTOR_PWM_IDLE_US);
    MOTOR_STATE[id - 1] = true;
  } else {
    motor_write_us(id, MOTOR_PWM_MIN_US); // 1000
    MOTOR_STATE[id - 1] = false;
  }
  return true;
}

bool encender_motor(uint8_t id) { return set_motor(id, true); }
bool apagar_motor(uint8_t id)   { return set_motor(id, false); }

bool toggle_motor(uint8_t id) {
  if (!motor_id_ok(id)) { Serial.printf("⚠️ Motor id inválido: %u\n", id); return false; }
  return set_motor(id, !MOTOR_STATE[id - 1]);
}

bool set_motor_speed(uint8_t id, int us) {
  if (!motor_id_ok(id)) { Serial.printf("⚠️ Motor id inválido: %u\n", id); return false; }
  motor_write_us(id, us);
  MOTOR_STATE[id - 1] = (clamp_us(us) > MOTOR_PWM_MIN_US);
  return true;
}

bool set_motor_on_with_speed(uint8_t id, int us) {
  int v = clamp_us(us);
  motor_write_us(id, v);
  MOTOR_STATE[id - 1] = (v > MOTOR_PWM_MIN_US);
  return true;
}

void encender_todos_los_motores() { for (uint8_t i = 1; i <= MOTOR_COUNT; ++i) encender_motor(i); }

void set_motores_ids_onoff(const uint8_t* ids, size_t count, bool on) {
  if (!ids) return;
  for (size_t k = 0; k < count; ++k) set_motor(ids[k], on);
}

void set_all_motors_speed(int us) {
  const int v = clamp_us(us);
  for (uint8_t i = 1; i <= MOTOR_COUNT; ++i) {
    motor_write_us(i, v);
    MOTOR_STATE[i - 1] = (v > MOTOR_PWM_MIN_US);
  }
  Serial.printf("ALL motors -> %d us\n", v);
}

void apagarMotores() {
  for (uint8_t i = 1; i <= motores_count(); ++i) {
    motor_write_us(i, MOTOR_PWM_MIN_US); // 1000
    MOTOR_STATE[i - 1] = false;
  }
}

void setupMotores()
{
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    mot1.attach(mot1_pin, 1000, 2000);
    mot2.attach(mot2_pin, 1000, 2000);
    mot3.attach(mot3_pin, 1000, 2000);
    mot4.attach(mot4_pin, 1000, 2000);
    delay(1000);

    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);

    mot1.attach(mot1_pin, 1000, 2000);
    delay(1000);
    mot1.setPeriodHertz(ESCfreq);
    delay(100);
    mot2.attach(mot2_pin, 1000, 2000);
    delay(1000);
    mot2.setPeriodHertz(ESCfreq);
    delay(100);
    mot3.attach(mot3_pin, 1000, 2000);
    delay(1000);
    mot3.setPeriodHertz(ESCfreq);
    delay(100);
    mot4.attach(mot4_pin, 1000, 2000);
    delay(1000);
    mot4.setPeriodHertz(ESCfreq);
    delay(100);

    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);
    Serial.println("Motores inicializados");
}

// === CONTROL A LOS MOTORES CON SATURACION COLECTIVA ===
void applyControl(float tau_x, float tau_y, float tau_z)
{
    int f[4];
    f[0] = InputThrottle - tau_x - tau_y - tau_z; // pwm1
    f[1] = InputThrottle - tau_x + tau_y + tau_z; // pwm2
    f[2] = InputThrottle + tau_x + tau_y - tau_z; // pwm3
    f[3] = InputThrottle + tau_x - tau_y + tau_z; // pwm4

    const float f_min = 1000.0;
    const float f_max = 1990.0;
    bool saturado = false;

    for (int i = 0; i < 4; i++)
    {
        if (f[i] < f_min || f[i] > f_max)
        {
            saturado = true;
            break;
        }
    }

    if (saturado)
    {
        float max_violation = 0;
        int j = 0;
        for (int i = 0; i < 4; i++)
        {
            float violation = 0;
            if (f[i] > f_max)
                violation = f[i] - f_max;
            else if (f[i] < f_min)
                violation = f_min - f[i];

            if (violation > max_violation)
            {
                max_violation = violation;
                j = i;
            }
        }

        float gamma = 1.0;
        if (f[j] > f_max)
            gamma = f_max / f[j];
        else if (f[j] < f_min)
            gamma = f_min / f[j];

        for (int i = 0; i < 4; i++)
        {
            f[i] *= gamma;
        }
    }

    // Recorte final por seguridad
    MotorInput1 = constrain(f[0], f_min, f_max);
    MotorInput2 = constrain(f[1], f_min, f_max);
    MotorInput3 = constrain(f[2], f_min, f_max);
    MotorInput4 = constrain(f[3], f_min, f_max);

    mot1.writeMicroseconds(round(MotorInput1));
    mot2.writeMicroseconds(round(MotorInput2));
    mot3.writeMicroseconds(round(MotorInput3));
    mot4.writeMicroseconds(round(MotorInput4));
}
