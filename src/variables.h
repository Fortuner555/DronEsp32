#ifndef VARIABLES_H
#define VARIABLES_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU6050.h>
#pragma once

extern volatile int modoActual;

// Define window_size before it is used
#define window_size 10 // Tamaño de la ventana

extern MPU6050 accelgyro;
extern volatile float RatePitch, RateRoll, RateYaw;
extern float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;
extern int pinLed;

extern int ESCfreq;

extern uint32_t LoopTimer;
extern float t;

extern volatile float AnglePitch_est;
extern volatile float AngleRoll_est;

extern Servo mot1;
extern Servo mot2;
extern Servo mot3;
extern Servo mot4;

extern const int mot1_pin;
extern const int mot2_pin;
extern const int mot3_pin;
extern const int mot4_pin;

// Variables de estado - OPTIMIZADO: quitado volatile innecesario
extern volatile float  phi_ref, theta_ref, psi_ref;

extern float integral_phi, integral_theta, integral_psi;

// === Variables para control avanzado ===
// Modo deslizante
extern float S_phi, S_theta, S_psi; // Superficies deslizantes
extern float lambda_sliding; // Parámetro de deslizamiento

// Feedforward
extern float ff_phi, ff_theta, ff_psi; // Términos feedforward

// Variables para la calibración
extern int buffersize;    // Cantidad de lecturas para promediar
extern int acel_deadzone; // Zona muerta del acelerómetro
extern int giro_deadzone; // Zona muerta del giroscopio

extern int16_t ax, ay, az, gx, gy, gz;
extern int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
extern int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

extern volatile float AngleRoll_est;
extern volatile float AnglePitch_est;
extern volatile float roll_rad, pitch_rad; // Mantener volatile - compartido entre tareas
extern volatile float gyroRoll_rad, gyroPitch_rad;
extern float tau_x, tau_y, tau_z;
extern float error_phi, error_theta, error_psi;

extern volatile uint32_t current_time;
extern volatile uint32_t last_channel_1;
extern volatile uint32_t last_channel_2;
extern volatile uint32_t last_channel_3;
extern volatile uint32_t last_channel_4;
extern volatile uint32_t last_channel_5;
extern volatile uint32_t last_channel_6;
extern volatile uint32_t timer_1;
extern volatile uint32_t timer_2;
extern volatile uint32_t timer_3;
extern volatile uint32_t timer_4;
extern volatile uint32_t timer_5;
extern volatile uint32_t timer_6;
extern volatile int ReceiverValue[6];
extern const int channel_1_pin;
extern const int channel_2_pin;
extern const int channel_3_pin;
extern const int channel_4_pin;
extern const int channel_5_pin;
extern const int channel_6_pin;

extern int ThrottleIdle;
extern int ThrottleCutOff;

// Kalman filters for angle mode
extern volatile float AccX, AccY, AccZ;                // Mantener volatile - compartido entre tareas
extern volatile float AngleRoll, AnglePitch, AngleYaw; // Mantener volatile - compartido entre tareas
extern float GyroXdps, GyroYdps, GyroZdps;             // OPTIMIZADO: solo uso local
// OPTIMIZADO: quitado volatile innecesario para mejor rendimiento
extern int DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
extern int InputRoll, InputThrottle, InputPitch, InputYaw;
extern int DesiredAngleRoll, DesiredAnglePitch, DesiredAngleYaw;
extern float ErrorAngleRoll, ErrorAnglePitch;
extern float PrevErrorAngleRoll, PrevErrorAnglePitch;
extern float PrevItermAngleRoll, PrevItermAnglePitch;

extern float complementaryAngleRoll;
extern float complementaryAnglePitch;

extern float MotorInput1, MotorInput2, MotorInput3, MotorInput4; // Solo salida de control

// Matriz de covarianza del error
extern float dt; // Paso de tiempo (ajustar según la frecuencia de muestreo, variable)

// Use TIME_STEP for macro-defined time step
extern float Q_angle; // Covarianza del ruido del proceso (ángulo)
extern float Q_gyro;  // Covarianza del ruido del proceso (giroscopio)
extern float R_angle; // Covarianza del ruido de medición (acelerómetro)

// Estado y matrices de covarianza para pitch
extern volatile float x_roll[2];
extern volatile float x_pitch[2];
extern float P_roll[2][2], P_pitch[2][2];
extern float residual_history_roll[window_size];
extern float residual_history_pitch[window_size];
extern int residual_index_roll, residual_index_pitch;
extern float R_angle_roll, R_angle_pitch;
extern float lambda_roll, lambda_pitch;

extern float accAngleRoll;  // Ángulo de roll (grados)
extern float accAnglePitch; // Ángulo de pitch (grados)
extern float gyroRateRoll;  // Tasa de giro en grados/segundo
extern float gyroRatePitch;

extern float accAngleY;
extern float accAngleX;

// === Configuración del sistema ===
extern const uint16_t LOOP_FREQ; // Frecuencia del loop en Hz
extern const float DT;           // Paso de tiempo
extern const uint32_t LOOP_US;   // Microsegundos por ciclo
extern const int IDLE_PWM;

extern float lambda;
extern float residual_history[window_size];
extern int residual_index;
extern float c_threshold;

extern float dt;
extern float Q_angle;
extern float Q_gyro;
extern float R_angle;
extern float P_roll[2][2];
extern float P_pitch[2][2];

// --- CALIBRATION OFFSETS ---
extern double accelOffsetX, accelOffsetY, accelOffsetZ;
extern double gyroXOffset, gyroYOffset, gyroZOffset;

// --- FILTER VARIABLES ---
extern double pitch, roll;

extern double Q_bias;
extern double R_measure;
extern double angle, bias, rate;
extern double P[2][2];

extern unsigned long lastTime;

struct Kalman
{
    double angle;   // Estimated angle
    double bias;    // Estimated bias
    double P[2][2]; // Error covariance matrix
};

extern Kalman kalmanRoll;
extern Kalman kalmanPitch;
extern Kalman kalmanYaw;

// Estados
extern float z;          // Altitud medida [m]
extern float dz;         // Velocidad vertical medida [m/s]
extern float z_ref;      // Altitud deseada [m]
extern float integral_z; // Integral del error de altitud

// Control
extern float T; // Empuje total deseado

// Declare global variables
extern float vel_z;
extern float error_z;

extern float magbias[3];
extern float magscale[3];

// Quaternion variables for orientation
extern float q0, q1, q2, q3;

// Euler angles in radians (converted from quaternions)
extern float yaw;

// Function declarations
float sat(float x, float epsilon);

extern float k1, k2, k3;
extern float g1, g2, g3;
extern float m1, m2, m3;


extern float Throttle_sal;

#endif // VARIABLES_H