#include <MPU6050.h>
#include "variables.h"

volatile int modoActual = 1;

// Define global variables
float vel_z = 0.0;
float error_z = 0.0;

MPU6050 accelgyro;

volatile float RatePitch = 0.0, RateRoll = 0.0, RateYaw = 0.0;

float RateCalibrationRoll = 0.27;
float RateCalibrationPitch = -0.85;
float RateCalibrationYaw = -2.09;
float AccXCalibration = 0.03;
float AccYCalibration = 0.01;
float AccZCalibration = -0.07;

int pinLed = 2;
int ESCfreq = 500;

volatile float AngleRoll_est;
volatile float AnglePitch_est;
float tau_x, tau_y, tau_z;
float error_phi, error_theta, error_psi;

int buffersize = 1000; // Cantidad de lecturas para promediar
int acel_deadzone = 8; // Zona muerta del acelerómetro
int giro_deadzone = 1; // Zona muerta del giroscopio

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// Variables para la calibración
uint32_t LoopTimer;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

int16_t ax, ay, az, gx, gy, gz;

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6];
const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

// Kalman filters for angle mode - OPTIMIZADO: quitado volatile innecesario
volatile float AccX, AccY, AccZ;                            // Mantener volatile - compartido entre tareas
volatile float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0; // Mantener volatile - compartido entre tareas
volatile float roll_rad = 0.0f, pitch_rad = 0.0f; // Mantener volatile - compartido entre tareas
volatile float gyroRoll_rad = 0.0f, gyroPitch_rad = 0.0f;
float GyroXdps, GyroYdps, GyroZdps;                         // OPTIMIZADO: solo uso local
int DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
int InputRoll, InputThrottle, InputPitch, InputYaw;
int DesiredAngleRoll, DesiredAnglePitch, DesiredAngleYaw;
float ErrorAngleRoll, ErrorAnglePitch;         // OPTIMIZADO: solo cálculos locales
float PrevErrorAngleRoll, PrevErrorAnglePitch; // OPTIMIZADO: solo cálculos locales
float PrevItermAngleRoll, PrevItermAnglePitch; // OPTIMIZADO: solo cálculos locales

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4; // OPTIMIZADO: solo salida de control

// Variables de estado
volatile float phi_ref = 0.0;  
volatile float theta_ref = 0.0;
volatile float psi_ref = 0.0;  
float integral_phi;   
float integral_theta; 
float integral_psi;   

// === Variables para control avanzado ===
// Modo deslizante
float S_phi, S_theta, S_psi; // Superficies deslizantes

// Feedforward
float ff_phi = 0, ff_theta = 0, ff_psi = 0; // Términos feedforward

float accAngleRoll;  // Ángulo de roll (grados)
float accAnglePitch; // Ángulo de pitch (grados)
float gyroRateRoll;
float gyroRatePitch;
float accAngleY;
float accAngleX;

float residual_history_roll[window_size] = {0};
float residual_history_pitch[window_size] = {0};
int residual_index_roll, residual_index_pitch;
float R_angle_roll, R_angle_pitch;
float lambda_roll, lambda_pitch;

// === Configuración del sistema ===
const uint16_t LOOP_FREQ = 100;               // Frecuencia del loop en Hz
const float DT = 1.0f / LOOP_FREQ;            // Paso de tiempo
const uint32_t LOOP_US = 1000000 / LOOP_FREQ; // Microsegundos por ciclo
const int IDLE_PWM = 1000;
float lambda = 0.96;
float lambda_sliding = 0.5f;
float residual_history[window_size] = {0};
int residual_index = 0;
float c_threshold = 0.01;

float dt = 0.006;       // Paso de tiempo (ajustar según la frecuencia de muestreo)
float Q_angle = 0.001f; // Covarianza del ruido del proceso (ángulo)
float Q_gyro = 0.003;   // Covarianza del ruido del proceso (giroscopio)
float R_angle = 0.03;   // Covarianza del ruido de medición (acelerómetro)

// --- CALIBRATION OFFSETS ---
double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
double gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// --- FILTER VARIABLES ---
double pitch = 0, roll = 0;
double Q_bias = 0.003f;
double R_measure = 0.03f;
double angle = 0.0f, bias = 0.0f, rate = 0;
double P[2][2] = {{0.0, 0.0}, {0.0, 0.0}};

Kalman kalmanRoll = {0, 0, {1, 0, 0, 1}};
Kalman kalmanPitch = {0, 0, {1, 0, 0, 1}};
Kalman kalmanYaw = {0, 0, {1, 0, 0, 1}};

unsigned long lastTime;
float T = 0.0f;

float magbias[3] = {0, 0, 0};  // Reemplaza estos valores tras calibrar
float magscale[3] = {1, 1, 1}; // Reemplaza estos valores tras calibrar

// Quaternion variables for orientation (initialize to identity quaternion)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Euler angles in radians (converted from quaternions)
float yaw = 0.0f;

// === UTILITY FUNCTIONS ===
float sat(float x, float epsilon) {
  if (x > epsilon) return 1.0;
  if (x < -epsilon) return -1.0;
  return x / epsilon;
}

float k1, k2, k3;
float g1, g2, g3;
float m1, m2, m3;

float Throttle_sal;