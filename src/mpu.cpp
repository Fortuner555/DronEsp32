#include <Arduino.h>
#include <Wire.h>
#include "variables.h"
#include "mpu.h"

// Direcciones I2C del MPU9250
#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C

// Registros del MPU9250
#define PWR_MGMT_1     0x6B
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define GYRO_XOUT_H    0x43
#define INT_PIN_CFG    0x37
#define USER_CTRL      0x6A

// Registros del magnetómetro AK8963
#define AK8963_WHO_AM_I  0x00
#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03
#define AK8963_CNTL      0x0A

int yawOffset = 0;
const int YAW_FILTER_SIZE = 5;
float yawHistory[YAW_FILTER_SIZE] = {0};
int yawIndex = 0;
const float MAG_THRESHOLD = 1500.0;

// Variable para umbral de compensación configurable
static float compensationThreshold = 2.0;

// Variables para el magnetómetro
float magX, magY, magZ;
float magCalibration[3] = {0, 0, 0};

// Función para el filtro de Kalman (roll)
double Kalman_filter(Kalman &kf, float newAngle, float newRate, float dt)
{
  // Predicción:
  double rate = newRate - kf.bias;
  kf.angle += dt * rate;

  // Actualización de la matriz de error
  kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + Q_angle);
  kf.P[0][1] -= dt * kf.P[1][1];
  kf.P[1][0] -= dt * kf.P[1][1];
  kf.P[1][1] += Q_bias * dt;

  // Medición:
  float S = kf.P[0][0] + R_measure;
  float K0 = kf.P[0][0] / S;
  float K1 = kf.P[1][0] / S;

  // Actualización con la medición (newAngle)
  float y = newAngle - kf.angle;
  kf.angle += K0 * y;
  kf.bias += K1 * y;

  // Actualizar la matriz de covarianza
  double P00_temp = kf.P[0][0];
  double P01_temp = kf.P[0][1];

  kf.P[0][0] -= K0 * P00_temp;
  kf.P[0][1] -= K0 * P01_temp;
  kf.P[1][0] -= K1 * P00_temp;
  kf.P[1][1] -= K1 * P01_temp;

  return kf.angle;
}

// Función para escribir registro del MPU9250 con verificación de error
void writeMPU9250Register(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  uint8_t error = Wire.endTransmission();
  if(error != 0) {
    Serial.print("Error I2C escribiendo registro 0x");
    Serial.print(reg, HEX);
    Serial.print(": ");
    Serial.println(error);
  }
  delay(1); // Pequeña pausa para estabilidad
}

// Función para leer registro del MPU9250 con verificación de error
uint8_t readMPU9250Register(uint8_t reg)
{
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(false);
  if(error != 0) {
    Serial.print("Error I2C en transmisión: ");
    Serial.println(error);
    return 0;
  }
  
  uint8_t bytesReceived = Wire.requestFrom(MPU9250_ADDRESS, 1);
  if(bytesReceived != 1) {
    Serial.print("Error: Solo se recibieron ");
    Serial.print(bytesReceived);
    Serial.println(" bytes");
    return 0;
  }
  
  return Wire.read();
}

// Función para escribir registro del magnetómetro AK8963
void writeAK8963Register(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// Función para leer registro del magnetómetro AK8963
uint8_t readAK8963Register(uint8_t reg)
{
  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDRESS, 1);
  return Wire.read();
}

// Inicializar magnetómetro AK8963 con verificación de errores
void initAK8963()
{
  Serial.println("Iniciando configuración del magnetómetro AK8963...");
  
  // Habilitar acceso al magnetómetro
  writeMPU9250Register(INT_PIN_CFG, 0x02);
  delay(100);
  
  // Verificar si el magnetómetro responde
  Wire.beginTransmission(AK8963_ADDRESS);
  uint8_t magError = Wire.endTransmission();
  
  if(magError != 0) {
    Serial.print("Error: No se puede comunicar con AK8963. Error I2C: ");
    Serial.println(magError);
    Serial.println("Continuando sin magnetómetro...");
    return;
  }
  
  // Verificar WHO_AM_I del magnetómetro
  uint8_t magWhoAmI = readAK8963Register(AK8963_WHO_AM_I);
  if(magWhoAmI == 0x48) {
    Serial.println("AK8963 detectado correctamente");
  } else {
    Serial.print("Advertencia: Respuesta inesperada del AK8963. WHO_AM_I = 0x");
    Serial.println(magWhoAmI, HEX);
  }
  
  // Reset del magnetómetro
  writeAK8963Register(AK8963_CNTL, 0x00);
  delay(100);
  
  // Entrar en modo Fuse ROM
  writeAK8963Register(AK8963_CNTL, 0x0F);
  delay(100);
  
  // Leer datos de calibración de fábrica
  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(0x10); // ASAX
  Wire.endTransmission(false);
  uint8_t bytesReceived = Wire.requestFrom(AK8963_ADDRESS, 3);
  
  if(bytesReceived == 3) {
    uint8_t rawData[3];
    for(int i = 0; i < 3; i++) {
      rawData[i] = Wire.read();
    }
    
    // Calcular valores de calibración
    magCalibration[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;
    magCalibration[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
    magCalibration[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;
    
    Serial.print("Calibración del magnetómetro - X: ");
    Serial.print(magCalibration[0], 3);
    Serial.print(", Y: ");
    Serial.print(magCalibration[1], 3);
    Serial.print(", Z: ");
    Serial.println(magCalibration[2], 3);
  } else {
    Serial.println("Error leyendo calibración del magnetómetro, usando valores por defecto");
    magCalibration[0] = 1.0f;
    magCalibration[1] = 1.0f;
    magCalibration[2] = 1.0f;
  }
  
  // Salir del modo Fuse ROM y configurar modo continuo
  writeAK8963Register(AK8963_CNTL, 0x00);
  delay(100);
  writeAK8963Register(AK8963_CNTL, 0x16); // 16-bit, 100Hz
  delay(100);
  
  Serial.println("Magnetómetro configurado");
}

// Leer datos del magnetómetro
void readMagnetometer()
{
  // Verificar si hay nuevos datos disponibles
  uint8_t st1 = readAK8963Register(AK8963_ST1);
  if(st1 & 0x01) {
    // Leer datos del magnetómetro
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(AK8963_XOUT_L);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8963_ADDRESS, 7);
    
    uint8_t rawData[7];
    for(int i = 0; i < 7; i++) {
      rawData[i] = Wire.read();
    }
    
    // Combinar datos LSB y MSB
    int16_t magXraw = ((int16_t)rawData[1] << 8) | rawData[0];
    int16_t magYraw = ((int16_t)rawData[3] << 8) | rawData[2];
    int16_t magZraw = ((int16_t)rawData[5] << 8) | rawData[4];
    
    // Aplicar calibración de fábrica
    magX = (float)magXraw * magCalibration[0];
    magY = (float)magYraw * magCalibration[1];
    magZ = (float)magZraw * magCalibration[2];
  }
}

void gyro_signals(void)
{
  // Configurar filtro paso bajo del acelerómetro
  writeMPU9250Register(CONFIG, 0x05);
  
  // Configurar rango del acelerómetro a ±8g
  writeMPU9250Register(ACCEL_CONFIG, 0x10);
  
  // Leer datos del acelerómetro
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);
  
  int16_t AccXLSB = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t AccYLSB = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t AccZLSB = (int16_t)(Wire.read() << 8 | Wire.read());
  
  // Configurar rango del giroscopio a ±500°/s
  writeMPU9250Register(GYRO_CONFIG, 0x08);
  
  // Leer datos del giroscopio
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);
  
  int16_t GyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t GyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  int16_t GyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

  // Aplicar offsets de calibración
  AccXLSB += ax_offset;
  AccYLSB += ay_offset;
  AccZLSB += az_offset;
  GyroX += gx_offset;
  GyroY += gy_offset;
  GyroZ += gz_offset;

  // Convertir datos del giroscopio (±500°/s range, 65.5 LSB/°/s)
  gyroRateRoll = (float)GyroX / 65.5f;
  gyroRatePitch = (float)GyroY / 65.5f;
  RateYaw = (float)GyroZ / 65.5f;

  // Convertir datos del acelerómetro (±8g range, 4096 LSB/g)
  AccX = (float)AccXLSB / 4096.0f;
  AccY = (float)AccYLSB / 4096.0f;
  AccZ = (float)AccZLSB / 4096.0f;

  // Cálculo del ángulo estimado a partir del acelerómetro
  AngleRoll_est = atan2f(AccY, sqrtf(AccX * AccX + AccZ * AccZ)) * 180.0f / PI;
  AnglePitch_est = -atan2f(AccX, sqrtf(AccY * AccY + AccZ * AccZ)) * 180.0f / PI;

  // Utiliza las tasas del giroscopio
  float gyroRateRoll_local = gyroRateRoll;
  float gyroRatePitch_local = gyroRatePitch;

  // Actualización del filtro de Kalman para cada eje
  AngleRoll = Kalman_filter(kalmanRoll, AngleRoll_est, gyroRateRoll_local, dt);
  AnglePitch = Kalman_filter(kalmanPitch, AnglePitch_est, gyroRatePitch_local, dt);
}

void loop_yaw()
{
  // Leer datos del magnetómetro
  readMagnetometer();
  
  // Calcular heading usando los datos del magnetómetro
  float heading = atan2(magY, magX);
  
  // Normalizar el heading entre 0 y 2*PI
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
}

void setupMPU()
{
  Serial.begin(115200);
  delay(1000); // Esperar que el serial esté listo
  
  // Inicializar I2C en pines 8 y 9 para ESP32-S3
  Wire.begin(21, 22); // SDA = GPIO8, SCL = GPIO9
  Wire.setClock(400000); // Reducir a 100kHz para mayor estabilidad
  
  Serial.println("Inicializando MPU9250...");
  delay(100);
  
  // Reset del MPU9250
  Serial.println("Reseteando MPU9250...");
  writeMPU9250Register(PWR_MGMT_1, 0x80);
  delay(200);
  
  // Despertar el MPU9250
  Serial.println("Despertando MPU9250...");
  writeMPU9250Register(PWR_MGMT_1, 0x00);
  delay(200);
  
  // Configuración básica
  writeMPU9250Register(PWR_MGMT_1, 0x01); // Usar PLL con referencia X gyro
  delay(100);
  
  // Configurar sample rate a 1kHz
  writeMPU9250Register(SMPLRT_DIV, 0x07);
  delay(10);
  
  // Configurar filtro paso bajo
  writeMPU9250Register(CONFIG, 0x00);
  delay(10);
  
  // Configurar giroscopio (±500°/s)
  writeMPU9250Register(GYRO_CONFIG, 0x08);
  delay(10);
  
  // Configurar acelerómetro (±8g)
  writeMPU9250Register(ACCEL_CONFIG, 0x10);
  delay(10);
  
  // Verificar comunicación
  uint8_t whoAmI = readMPU9250Register(0x75);
  if(whoAmI == 0x71 || whoAmI == 0x70 || whoAmI == 0x68) {
    Serial.print("MPU9250 conectado correctamente. WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);
  } else {
    Serial.print("Advertencia: Respuesta inesperada. WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);
    Serial.println("Continuando con la inicialización...");
  }
  
  // Inicializar magnetómetro
  initAK8963();
  
  delay(2000); // Esperar estabilización

  calibrateSensors();
  Serial.println("Calibración completada.");
}

// === CALIBRACIÓN DEL MPU9250 ===
void calibrateSensors()
{
  Serial.println("\nCalibrando sensores...");
  Serial.println("IMPORTANTE: Mantén el sensor completamente inmóvil durante la calibración");

  // Para el MPU9250, simplificaremos la calibración
  // No usaremos registros de offset internos, sino que calcularemos offsets de software

  meansensors();
  Serial.println("\nCalculando offsets...");
  
  // Calcular offsets simples basados en la primera lectura
  ax_offset = -mean_ax;
  ay_offset = -mean_ay;
  az_offset = 4096  - mean_az; // Para 1g en el eje Z
  
  gx_offset = -mean_gx;
  gy_offset = -mean_gy;
  gz_offset = -mean_gz;
  
  Serial.println("--- Offsets calculados ---");
  Serial.print("Acelerómetro - X: "); Serial.print(ax_offset);
  Serial.print(", Y: "); Serial.print(ay_offset);
  Serial.print(", Z: "); Serial.println(az_offset);
  Serial.print("Giroscopio - X: "); Serial.print(gx_offset);
  Serial.print(", Y: "); Serial.print(gy_offset);
  Serial.print(", Z: "); Serial.println(gz_offset);

  // Verificar calibración
  Serial.println("\nVerificando calibración...");
  meansensors();
  
  // Aplicar offsets de software para verificar
  int16_t ax_cal = mean_ax + ax_offset;
  int16_t ay_cal = mean_ay + ay_offset;
  int16_t az_cal = mean_az + az_offset;
  int16_t gx_cal = mean_gx + gx_offset;
  int16_t gy_cal = mean_gy + gy_offset;
  int16_t gz_cal = mean_gz + gz_offset;
  
  Serial.print("Valores calibrados - AX: "); Serial.print(ax_cal);
  Serial.print(", AY: "); Serial.print(ay_cal);
  Serial.print(", AZ: "); Serial.print(az_cal);
  Serial.print(" | GX: "); Serial.print(gx_cal);
  Serial.print(", GY: "); Serial.print(gy_cal);
  Serial.print(", GZ: "); Serial.println(gz_cal);

  Serial.println("Calibración completada.");
}

void meansensors()
{
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  
  Serial.println("Leyendo muestras para calibración...");
  
  while (i < (buffersize + 101))
  {
    // Leer acelerómetro con verificación de errores
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(ACCEL_XOUT_H);
    uint8_t error = Wire.endTransmission(false);
    
    if(error == 0) {
      uint8_t bytesReceived = Wire.requestFrom(MPU9250_ADDRESS, 6);
      
      if(bytesReceived == 6) {
        ax = (int16_t)(Wire.read() << 8 | Wire.read());
        ay = (int16_t)(Wire.read() << 8 | Wire.read());
        az = (int16_t)(Wire.read() << 8 | Wire.read());
      } else {
        Serial.print("Error leyendo acelerómetro, bytes: ");
        Serial.println(bytesReceived);
        continue;
      }
    } else {
      Serial.print("Error I2C acelerómetro: ");
      Serial.println(error);
      continue;
    }
    
    // Leer giroscopio con verificación de errores
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(GYRO_XOUT_H);
    error = Wire.endTransmission(false);
    
    if(error == 0) {
      uint8_t bytesReceived = Wire.requestFrom(MPU9250_ADDRESS, 6);
      
      if(bytesReceived == 6) {
        gx = (int16_t)(Wire.read() << 8 | Wire.read());
        gy = (int16_t)(Wire.read() << 8 | Wire.read());
        gz = (int16_t)(Wire.read() << 8 | Wire.read());
      } else {
        Serial.print("Error leyendo giroscopio, bytes: ");
        Serial.println(bytesReceived);
        continue;
      }
    } else {
      Serial.print("Error I2C giroscopio: ");
      Serial.println(error);
      continue;
    }
    
    if (i > 100 && i <= (buffersize + 100))
    {
      buff_ax += ax;
      buff_ay += ay;
      buff_az += az;
      buff_gx += gx;
      buff_gy += gy;
      buff_gz += gz;
      
      // Mostrar progreso cada 50 muestras
      if((i - 100) % 50 == 0) {
        Serial.print("Progreso: ");
        Serial.print(i - 100);
        Serial.print("/");
        Serial.println(buffersize);
      }
    }
    i++;
    delay(2);
  }

  mean_ax = buff_ax / buffersize;
  mean_ay = buff_ay / buffersize;
  mean_az = buff_az / buffersize;
  mean_gx = buff_gx / buffersize;
  mean_gy = buff_gy / buffersize;
  mean_gz = buff_gz / buffersize;
  
  Serial.println("Muestras obtenidas correctamente");
  Serial.print("Promedios - AX: "); Serial.print(mean_ax);
  Serial.print(", AY: "); Serial.print(mean_ay);
  Serial.print(", AZ: "); Serial.println(mean_az);
  Serial.print("GX: "); Serial.print(mean_gx);
  Serial.print(", GY: "); Serial.print(mean_gy);
  Serial.print(", GZ: "); Serial.println(mean_gz);
}

void calibration()
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (4096  - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  int iteration = 0;
  const int MAX_ITERATIONS = 10; // Límite de iteraciones para evitar bucle infinito

  while (iteration < MAX_ITERATIONS)
  {
    iteration++;
    Serial.print("Iteración de calibración: ");
    Serial.println(iteration);
    
    int ready = 0;
    
    // Aplicar offsets temporalmente usando registros de offset del MPU9250
    // Para el MPU9250, usamos un enfoque más simple
    
    meansensors();
    
    Serial.print("Valores actuales - AX: "); Serial.print(mean_ax);
    Serial.print(", AY: "); Serial.print(mean_ay);
    Serial.print(", AZ: "); Serial.print(mean_az);
    Serial.print(" | GX: "); Serial.print(mean_gx);
    Serial.print(", GY: "); Serial.print(mean_gy);
    Serial.print(", GZ: "); Serial.println(mean_gz);

    // Verificar acelerómetro
    if (abs(mean_ax) <= acel_deadzone) {
      ready++;
      Serial.println("AX calibrado ✓");
    } else {
      ax_offset -= mean_ax / acel_deadzone;
      Serial.print("Ajustando AX offset: ");
      Serial.println(ax_offset);
    }

    if (abs(mean_ay) <= acel_deadzone) {
      ready++;
      Serial.println("AY calibrado ✓");
    } else {
      ay_offset -= mean_ay / acel_deadzone;
      Serial.print("Ajustando AY offset: ");
      Serial.println(ay_offset);
    }

    if (abs(4096  - mean_az) <= acel_deadzone) {
      ready++;
      Serial.println("AZ calibrado ✓");
    } else {
      az_offset += (4096  - mean_az) / acel_deadzone;
      Serial.print("Ajustando AZ offset: ");
      Serial.println(az_offset);
    }

    // Verificar giroscopio
    if (abs(mean_gx) <= giro_deadzone) {
      ready++;
      Serial.println("GX calibrado ✓");
    } else {
      gx_offset -= mean_gx / (giro_deadzone + 1);
      Serial.print("Ajustando GX offset: ");
      Serial.println(gx_offset);
    }

    if (abs(mean_gy) <= giro_deadzone) {
      ready++;
      Serial.println("GY calibrado ✓");
    } else {
      gy_offset -= mean_gy / (giro_deadzone + 1);
      Serial.print("Ajustando GY offset: ");
      Serial.println(gy_offset);
    }

    if (abs(mean_gz) <= giro_deadzone) {
      ready++;
      Serial.println("GZ calibrado ✓");
    } else {
      gz_offset -= mean_gz / (giro_deadzone + 1);
      Serial.print("Ajustando GZ offset: ");
      Serial.println(gz_offset);
    }

    Serial.print("Ejes calibrados: ");
    Serial.print(ready);
    Serial.println("/6");
    
    if (ready == 6) {
      Serial.println("¡Calibración completada exitosamente!");
      break;
    }
    
    if (iteration == MAX_ITERATIONS) {
      Serial.println("Calibración terminada por límite de iteraciones");
      Serial.println("Los valores actuales serán utilizados");
    }
  }
  
  // Mostrar offsets finales
  Serial.println("--- Offsets finales ---");
  Serial.print("AX: "); Serial.print(ax_offset);
  Serial.print(", AY: "); Serial.print(ay_offset);
  Serial.print(", AZ: "); Serial.println(az_offset);
  Serial.print("GX: "); Serial.print(gx_offset);
  Serial.print(", GY: "); Serial.print(gy_offset);
  Serial.print(", GZ: "); Serial.println(gz_offset);
}