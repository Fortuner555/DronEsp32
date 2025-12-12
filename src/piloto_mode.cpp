#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include <esp_task_wdt.h>
#include "mpu.h"
#include "motores.h"

// ============================================================================
//                          CdgfONFIGURACIÓgdhdhdhfhhfN GENERAL
// ============================================================================
#define ANGLES_ARE_DEGREES 1              // 1 si los ángulos y tasas vienen en grados

// Límites de pulsos para ESC/motores
static const int   US_MIN   = 1100;       // Pulso mínimo (µs)
static const int   US_MAX   = 1900;       // Pulso máximo (µs)
static int         US_HOVER = 1500;       // Pulso para hover (calibrar según dron)

// Geometría del dron (configuración X)
static const float DRON_L   = 0.227f;     // Distancia centro→motor (m)
static const float DRON_LARM= DRON_L * 0.70710678f; // Brazo efectivo para X (L/√2)

// Ganancias de mezcla (conversión torque→PWM)
static float KTX = 650.0f;   // [µs/(N·m)] para roll
static float KTY = 650.0f;   // [µs/(N·m)] para pitch
static float KTZ = 200.0f;   // [µs/(N·m)] para yaw (no usado actualmente)

// Parámetros (comentados como no usados actualmente)
static const float TORQUE_MAX = 100.8f;     // (sin efecto ahora)
static const float MAX_DTAU   = 0.05f;      // (sin efecto ahora)

// Parámetros de control activos
static const float REF_TAU    = 0.8f;       // Constante tiempo para suavizado de referencias (s)
static const float MAX_REF_RATE = 2.0f;     // Tasa máxima de referencia (rad/s)
static const float RATE_LP_A  = 0.2f;       // Coeficiente filtro pasa-bajas para tasas (0..1)

// ============================================================================
//                  ESTRUCTURA DEL CONTROLADOR BACKSTEPPING
// ============================================================================
/**
 * @struct BacksteppingController
 * @brief Estructura que contiene todos los estados y parámetros del controlador
 */
struct BacksteppingController {
    // Estados de referencia anteriores
    float phi_ref_prev = 0;      // Referencia anterior de roll
    float theta_ref_prev = 0;    // Referencia anterior de pitch
    
    // Estados de control anteriores
    float torque_x_prev = 0;     // Torque anterior en eje X
    float torque_y_prev = 0;     // Torque anterior en eje Y
    
    // Ganancias del controlador
    float k1_phi = 2.5;          // Ganancia backstepping capa 1 para roll
    float k2_phi = 2.0;          // Ganancia backstepping capa 2 para roll
    float k1_theta = 2.5;        // Ganancia backstepping capa 1 para pitch
    float k2_theta = 2.0;        // Ganancia backstepping capa 2 para pitch
    
    // Parámetros estimados
    float theta2_roll_hat = 0.25;   // Estimación coeficiente amortiguamiento roll
    float theta2_pitch_hat = 0.25;  // Estimación coeficiente amortiguamiento pitch
    float Iyy_Izz_hat = 0;          // Estimación relación de inercias (Iyy-Izz)/Ixx
    float Izz_Ixx_hat = 0;          // Estimación relación de inercias (Izz-Ixx)/Iyy
    
    // Bandera de inicialización
    bool initialized = false;
};

// Instancia global del controlador
BacksteppingController bs_controller;

// ============================================================================
//                      CONSTANTES ADICIONALES
// ============================================================================
const float TAU_REF = 0.8f;           // Constante tiempo para referencias (alternativa)
const float MAX_CHANGE = 0.05f;       // Cambio máximo permitido (no usado actualmente)
const float MAX_REF_RATE_OLD = 1.5f;  // Tasa máxima referencia (alternativa)

// ============================================================================
//                      PARÁMETROS FÍSICOS DEL DRON
// ============================================================================
const float DRON_MASS = 1.1f;         // Masa total del dron (kg)
const float DRON_L_dummy = 0.227f;    // Distancia centro-motor para cálculo inercias (m)
const float DRON_H = 0.05f;           // Altura del cuadro (m)
const float MOTOR_MASS = 0.075f;      // Masa por motor (kg)

// ============================================================================
//                      FUNCIONES AUXILIARES
// ============================================================================

/**
 * @brief Limita un valor entre mínimo y máximo
 * @param v Valor a limitar
 * @param lo Límite inferior
 * @param hi Límite superior
 * @return Valor limitado
 */
static inline float clampf(float v, float lo, float hi) { 
    return v < lo ? lo : (v > hi ? hi : v); 
}

/**
 * @brief Calcula los momentos de inercia del dron
 * @param Ixx Momento de inercia en X (salida)
 * @param Iyy Momento de inercia en Y (salida)
 * @param Izz Momento de inercia en Z (salida)
 * 
 * Modela el dron como placa rectangular con 4 masas puntuales (motores)
 */
void calculateMomentsOfInertia(float& Ixx, float& Iyy, float& Izz) {
    // Inercia del cuadro (aproximación como placa delgada)
    Ixx = (1.0f/12.0f) * DRON_MASS * DRON_H * DRON_H +
          0.5f * DRON_MASS * DRON_L_dummy * DRON_L_dummy +
          4.0f * MOTOR_MASS * DRON_L_dummy * DRON_L_dummy;

    Iyy = Ixx;  // Simétrico para configuración X

    Izz = (1.0f/12.0f) * DRON_MASS * (DRON_L_dummy * DRON_L_dummy + DRON_H * DRON_H) +
          4.0f * MOTOR_MASS * DRON_L_dummy * DRON_L_dummy;
}

/**
 * @brief Suaviza una referencia usando filtro de primer orden
 * @param current_ref Referencia actual
 * @param prev_ref Referencia anterior (se actualiza)
 * @param dt Paso de tiempo (s)
 * @param tau Constante tiempo del filtro (s)
 * @return Referencia suavizada
 */
float smoothReference(float current_ref, float& prev_ref, float dt, float tau) {
    float alpha = expf(-dt / tau);  // Factor de suavizado
    return alpha * prev_ref + (1.0f - alpha) * current_ref;
}

/**
 * @brief Aplica límites a un valor
 * @param value Valor a limitar (referencia, se modifica)
 * @param min_val Límite mínimo
 * @param max_val Límite máximo
 */
void applyLimits(float& value, float min_val, float max_val) {
    value = clampf(value, min_val, max_val);
}

// ============================================================================
//                      FUNCIÓN DE INICIALIZACIÓN
// ============================================================================

/**
 * @brief Inicializa el controlador Backstepping Adaptativo
 * 
 * Configura valores iniciales y muestra mensaje por serial
 */
void setup_pilote_mode() {
    // Inicializar estados del controlador
    bs_controller.phi_ref_prev = 0;
    bs_controller.theta_ref_prev = 0;
    bs_controller.torque_x_prev = 0;
    bs_controller.torque_y_prev = 0;

    // Configurar ganancias iniciales (valores más razonables)
    bs_controller.k1_phi = 4.0f;    // Ganancia capa 1 roll
    bs_controller.k2_phi = 3.0f;    // Ganancia capa 2 roll
    bs_controller.k1_theta = 4.0f;  // Ganancia capa 1 pitch
    bs_controller.k2_theta = 3.0f;  // Ganancia capa 2 pitch

    // Inicializar parámetros estimados
    bs_controller.theta2_roll_hat = 0.25f;   // Coeficiente amortiguamiento roll
    bs_controller.theta2_pitch_hat = 0.25f;  // Coeficiente amortiguamiento pitch
    bs_controller.Iyy_Izz_hat = 0;           // Relación de inercias
    bs_controller.Izz_Ixx_hat = 0;           // Relación de inercias

    // Marcar como inicializado
    bs_controller.initialized = true;

    // Mensaje informativo
    Serial.println("==================================================");
    Serial.println("Controlador Backstepping Adaptativo Inicializado");
    Serial.println("Configuración: pitch/roll SIN límites de torque");
    Serial.println("==================================================");
}

// ============================================================================
//                      LOOP PRINCIPAL DE CONTROL
// ============================================================================

/**
 * @brief Bucle principal de control (ejecutado periódicamente)
 * @param dt Paso de tiempo desde última iteración (s)
 * 
 * Implementa controlador Backstepping adaptativo en dos etapas:
 * 1. Diseño de ley de control mediante backstepping
 * 2. Adaptación en línea de parámetros desconocidos
 */
void loop_pilote_mode(float dt) {
    // ==================================================
    // 1. VALIDACIÓN DEL PASO DE TIEMPO
    // ==================================================
    if (dt <= 0 || dt > 0.05f) {
        return;  // dt inválido, saltar iteración
    }

    // ==================================================
    // 2. LECTURA Y PROCESAMIENTO DE SENSORES IMU
    // ==================================================
    constexpr float DEG2RAD = 3.14159265359f / 180.0f;
    
    // Leer ángulos y tasas del IMU
    float phi   = AngleRoll;     // Ángulo de roll
    float theta = AnglePitch;    // Ángulo de pitch
    float p     = gyroRateRoll;  // Tasa de roll
    float q     = gyroRatePitch; // Tasa de pitch
    float r     = RateYaw;       // Tasa de yaw
    
    // Conversión a radianes si es necesario
#if ANGLES_ARE_DEGREES
    phi   *= DEG2RAD;
    theta *= DEG2RAD;
    p     *= DEG2RAD;
    q     *= DEG2RAD;
    r     *= DEG2RAD;
#endif
    
    // ==================================================
    // 3. FILTRADO DE TASAS ANGULARES
    // ==================================================
    static float p_f = 0.0f, q_f = 0.0f;  // Estados del filtro
    
    // Filtro pasa-bajas simple para reducir ruido
    p_f = (1.0f - RATE_LP_A) * p_f + RATE_LP_A * p;
    q_f = (1.0f - RATE_LP_A) * q_f + RATE_LP_A * q;
    
    // ==================================================
    // 4. CONFIGURACIÓN DE THROTTLE BASE
    // ==================================================
    InputThrottle = 1500;  // Pulso para hover (ajustar según dron real)
    
    // ==================================================
    // 5. GENERACIÓN Y SUAVIZADO DE REFERENCIAS
    // ==================================================
    float phi_ref_raw   = 0.0f;  // Referencia cruda de roll (0 = nivelado)
    float theta_ref_raw = 0.0f;  // Referencia cruda de pitch (0 = nivelado)
    
    // Estados para suavizado de referencias
    static float phi_ref_s = 0.0f, theta_ref_s = 0.0f;
    static bool init_ref = false;
    
    // Inicialización en primera iteración
    if (!init_ref) {
        phi_ref_s = phi_ref_raw;
        theta_ref_s = theta_ref_raw;
        init_ref = true;
    }
    
    // Filtro de primer orden para suavizar referencias
    float alpha = expf(-dt / REF_TAU);
    phi_ref_s   = alpha * phi_ref_s   + (1.0f - alpha) * phi_ref_raw;
    theta_ref_s = alpha * theta_ref_s + (1.0f - alpha) * theta_ref_raw;
    
    // ==================================================
    // 6. CÁLCULO DE REFERENCIAS DE VELOCIDAD
    // ==================================================
    static float phi_ref_prev = 0.0f, theta_ref_prev = 0.0f;
    
    // Derivada numérica para obtener referencias de velocidad
    float p_ref = (phi_ref_s - phi_ref_prev) / dt;      // Referencia velocidad roll
    float q_ref = (theta_ref_s - theta_ref_prev) / dt;  // Referencia velocidad pitch
    
    // Limitar tasa de cambio de referencias
    p_ref = clampf(p_ref, -MAX_REF_RATE, MAX_REF_RATE);
    q_ref = clampf(q_ref, -MAX_REF_RATE, MAX_REF_RATE);
    
    // Actualizar valores anteriores
    phi_ref_prev   = phi_ref_s;
    theta_ref_prev = theta_ref_s;
    
    // ==================================================
    // 7. CÁLCULO DE MOMENTOS DE INERCIA
    // ==================================================
    float Ixx, Iyy, Izz;
    calculateMomentsOfInertia(Ixx, Iyy, Izz);
    
    // ==================================================
    // 8. IMPLEMENTACIÓN DEL CONTROLADOR BACKSTEPPING
    // ==================================================
    
    // Referencias a parámetros del controlador (para legibilidad)
    float& k1_phi  = bs_controller.k1_phi;
    float& k2_phi  = bs_controller.k2_phi;
    float& k1_theta= bs_controller.k1_theta;
    float& k2_theta= bs_controller.k2_theta;
    float& th2_r   = bs_controller.theta2_roll_hat;
    float& th2_p   = bs_controller.theta2_pitch_hat;
    float& IyyIzz  = bs_controller.Iyy_Izz_hat;
    float& IzzIxx  = bs_controller.Izz_Ixx_hat;
    
    // --------------------------------------------------
    // 8.1. CÁLCULO DE ERRORES
    // --------------------------------------------------
    float e_phi   = (phi - phi_ref_s);      // Error de roll
    float e_theta = (theta - theta_ref_s);  // Error de pitch
    
    // --------------------------------------------------
    // 8.2. BACKSTEPPING PARA ROLL (2 ETAPAS)
    // --------------------------------------------------
    // Etapa 1: Control de posición angular
    float z1_phi     = e_phi;                     // Variable de error etapa 1
    float alpha_phi  = -k1_phi * z1_phi + p_ref;  // Función estabilizante
    float z2_phi     = p_f - alpha_phi;           // Variable de error etapa 2
    
    // Derivada de la función estabilizante
    float alpha_phi_dot = -k1_phi * (p_f - p_ref);
    
    // --------------------------------------------------
    // 8.3. BACKSTEPPING PARA PITCH (2 ETAPAS)
    // --------------------------------------------------
    // Etapa 1: Control de posición angular
    float z1_theta     = e_theta;                     // Variable de error etapa 1
    float alpha_theta  = -k1_theta * z1_theta + q_ref; // Función estabilizante
    float z2_theta     = q_f - alpha_theta;           // Variable de error etapa 2
    
    // Derivada de la función estabilizante
    float alpha_theta_dot = -k1_theta * (q_f - q_ref);
    
    // --------------------------------------------------
    // 8.4. CÁLCULO DE TÉRMINOS DE ACOPLAMIENTO Y AMORTIGUAMIENTO
    // --------------------------------------------------
    // Acoplamiento giroscópico: (Iyy-Izz)/Ixx * q * r
    float coupling_roll  = IyyIzz * q_f * r;
    
    // Amortiguamiento aerodinámico: θ₂ * p
    float damping_roll   = th2_r * p_f;
    
    // Acoplamiento giroscópico: (Izz-Ixx)/Iyy * p * r
    float coupling_pitch = IzzIxx * p_f * r;
    
    // Amortiguamiento aerodinámico: θ₂ * q
    float damping_pitch  = th2_p * q_f;
    
    // --------------------------------------------------
    // 8.5. LEY DE CONTROL FINAL (ECUACIONES DINÁMICAS)
    // --------------------------------------------------
    // τ = I * (α_dot - k₂*z₂ - z₁) - acoplamiento - amortiguamiento
    // Factor de escala empírico
    const float TORQUE_SCALER = 120.0f;
    
    // Torque para roll (eje X)
    tau_x = Ixx * (alpha_phi_dot - k2_phi * z2_phi - z1_phi) - coupling_roll - damping_roll;
    
    // Torque para pitch (eje Y)
    tau_y = Iyy * (alpha_theta_dot - k2_theta * z2_theta - z1_theta) - coupling_pitch - damping_pitch;
    
    // ==================================================
    // 9. ADAPTACIÓN DE PARÁMETROS (LEY DE ADAPTACIÓN)
    // ==================================================
    
    // Tasas de adaptación (ganancias de aprendizaje)
    float gamma_k1 = 0.05f;   // Para k1
    float gamma_k2 = 0.04f;   // Para k2
    float gamma_th = 0.03f;   // Para coeficientes de amortiguamiento
    float gamma_c  = 0.015f;  // Para coeficientes de acoplamiento
    
    // Adaptación de ganancias k1, k2 (solo si el error es significativo)
    if (fabsf(z1_phi) > 0.03f) {
        k1_phi += gamma_k1 * z1_phi * e_phi * dt;
    }
    
    if (fabsf(z2_phi) > 0.06f) {
        k2_phi += gamma_k2 * z2_phi * z2_phi * dt;
    }
    
    if (fabsf(z1_theta) > 0.03f) {
        k1_theta += gamma_k1 * z1_theta * e_theta * dt;
    }
    
    if (fabsf(z2_theta) > 0.06f) {
        k2_theta += gamma_k2 * z2_theta * z2_theta * dt;
    }
    
    // Adaptación de coeficientes de amortiguamiento
    th2_r += gamma_th * z2_phi * p_f * dt;
    th2_p += gamma_th * z2_theta * q_f * dt;
    
    // Adaptación de coeficientes de acoplamiento (solo si hay acoplamiento significativo)
    if (fabsf(q_f * r) > 0.01f) {
        IyyIzz += gamma_c * z2_phi * q_f * r * dt;
    }
    
    if (fabsf(p_f * r) > 0.01f) {
        IzzIxx += gamma_c * z2_theta * p_f * r * dt;
    }
    
    // ==================================================
    // 10. LIMITACIÓN DE PARÁMETROS ADAPTATIVOS
    // ==================================================
    // Asegurar que los parámetros se mantengan en rangos razonables
    
    // Ganancias del controlador
    applyLimits(k1_phi,   1.0f, 8.0f);
    applyLimits(k2_phi,   0.5f, 6.0f);
    applyLimits(k1_theta, 1.0f, 8.0f);
    applyLimits(k2_theta, 0.5f, 6.0f);
    
    // Coeficientes de amortiguamiento
    applyLimits(th2_r, 0.05f, 0.8f);
    applyLimits(th2_p, 0.05f, 0.8f);
    
    // Coeficientes de acoplamiento
    applyLimits(IyyIzz, -0.04f, 0.04f);
    applyLimits(IzzIxx, -0.04f, 0.04f);
    
    // ==================================================
    // 11. APLICACIÓN DEL CONTROL A LOS MOTORES
    // ==================================================
    float tau_z = 0.0f;  // Torque de yaw (desactivado por ahora)
    
    // Escalar torques (conversión a unidades del mixer)
    tau_x *= TORQUE_SCALER;
    tau_y *= TORQUE_SCALER;
    
    // Aplicar torques a los motores (función en motores.h)
    applyControl(tau_x, tau_y, tau_z);
}