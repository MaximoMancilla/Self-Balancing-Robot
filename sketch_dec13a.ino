/*
===========================================================
 Project:   Self-Balancing Robot (ESP32 + MPU6050)
 Author:    Máximo Mancilla
 Date:      2026
 Description:
   Two-wheeled self-balancing robot using:
   - ESP32
   - MPU6050 IMU
   - DRV8833 motor driver

   Control strategy:
     • Complementary filter for angle estimation
     • PD control on tilt
     • Velocity damping term (state feedback)
     • Deadzone compensation per motor
     • PWM slew rate limiting
===========================================================
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/* ================= HARDWARE CONFIG ================= */

// DRV8833 motor pins
constexpr int PIN_L_IN1 = 25;
constexpr int PIN_L_IN2 = 26;
constexpr int PIN_R_IN1 = 27;
constexpr int PIN_R_IN2 = 14;

// PWM settings
constexpr int PWM_FREQ = 1000;
constexpr int PWM_RES  = 8;
constexpr int MAX_PWM  = 190;

// Minimum PWM per motor/direction (from calibration)
constexpr int MIN_PWM_L_FWD = 78;
constexpr int MIN_PWM_L_REV = 78;
constexpr int MIN_PWM_R_FWD = 80;
constexpr int MIN_PWM_R_REV = 78;

// Slew rate limit
constexpr float MAX_PWM_STEP = 30.0;

// IMU
Adafruit_MPU6050 mpu;

/* ================= CONTROL TUNING ================= */

float targetAngle = -2.3;

float Kp = 7.8;
float Kd = 0.6;
float Kv = 0.4;

constexpr float FALL_ANGLE = 30.0;
constexpr float alpha = 0.98;   // complementary filter weight

/* ================= STATE VARIABLES ================= */

float angle = 0.0;
float velocity = 0.0;

float lastPWM_L = 0.0;
float lastPWM_R = 0.0;

unsigned long lastTime = 0;

/* =================================================== */
/* ================= HELPER FUNCTIONS ================= */
/* =================================================== */

void stopMotors() {
  ledcWrite(PIN_L_IN1, 0);
  ledcWrite(PIN_L_IN2, 0);
  ledcWrite(PIN_R_IN1, 0);
  ledcWrite(PIN_R_IN2, 0);

  lastPWM_L = 0;
  lastPWM_R = 0;
}

float computeAccelerometerAngle(sensors_event_t &acc) {
  return atan2(
    -acc.acceleration.x,
    sqrt(acc.acceleration.y * acc.acceleration.y +
         acc.acceleration.z * acc.acceleration.z)
  ) * 180.0 / PI;
}

void applySlewLimiter(int &pwmL, int &pwmR) {
  float deltaL = pwmL - lastPWM_L;
  if (deltaL >  MAX_PWM_STEP) pwmL = lastPWM_L + MAX_PWM_STEP;
  if (deltaL < -MAX_PWM_STEP) pwmL = lastPWM_L - MAX_PWM_STEP;

  float deltaR = pwmR - lastPWM_R;
  if (deltaR >  MAX_PWM_STEP) pwmR = lastPWM_R + MAX_PWM_STEP;
  if (deltaR < -MAX_PWM_STEP) pwmR = lastPWM_R - MAX_PWM_STEP;

  lastPWM_L = pwmL;
  lastPWM_R = pwmR;
}

void driveMotors(float control) {

  float pwmMag = abs(control);

  if (pwmMag < 5.0) {
    stopMotors();
    return;
  }

  int pwmL, pwmR;

  if (control > 0) {  // Forward
    pwmL = MIN_PWM_L_FWD + pwmMag;
    pwmR = MIN_PWM_R_FWD + pwmMag;
  } else {            // Reverse
    pwmL = MIN_PWM_L_REV + pwmMag;
    pwmR = MIN_PWM_R_REV + pwmMag;
  }

  pwmL = constrain(pwmL, 0, MAX_PWM);
  pwmR = constrain(pwmR, 0, MAX_PWM);

  applySlewLimiter(pwmL, pwmR);

  if (control > 0) {
    ledcWrite(PIN_L_IN1, pwmL);
    ledcWrite(PIN_L_IN2, 0);
    ledcWrite(PIN_R_IN1, pwmR);
    ledcWrite(PIN_R_IN2, 0);
  } else {
    ledcWrite(PIN_L_IN1, 0);
    ledcWrite(PIN_L_IN2, pwmL);
    ledcWrite(PIN_R_IN1, 0);
    ledcWrite(PIN_R_IN2, pwmR);
  }
}

/* =================================================== */
/* ====================== SETUP ====================== */
/* =================================================== */

void setup() {

  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not detected");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);

  ledcAttach(PIN_L_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_L_IN2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_R_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_R_IN2, PWM_FREQ, PWM_RES);

  // IMU Calibration
  float sum = 0;
  for (int i = 0; i < 300; i++) {
    sensors_event_t acc, gyro, temp;
    mpu.getEvent(&acc, &gyro, &temp);
    sum += computeAccelerometerAngle(acc);
    delay(3);
  }

  angle = sum / 300.0;
  lastTime = micros();

  Serial.println("System Ready — Hold upright and release");
}

/* =================================================== */
/* ======================= LOOP ====================== */
/* =================================================== */

void loop() {

  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  lastTime = now;

  if (dt <= 0 || dt > 0.03) return;

  /* ===== Angle Estimation (Complementary Filter) ===== */

  float accAngle = computeAccelerometerAngle(acc);
  float gyroRate = gyro.gyro.y * 180.0 / PI;

  angle = alpha * (angle + gyroRate * dt)
        + (1.0 - alpha) * accAngle;

  /* ===== Safety Cutoff ===== */

  if (abs(angle - targetAngle) > FALL_ANGLE) {
    stopMotors();
    velocity = 0;
    return;
  }

  /* ===== Control Law ===== */

  float error = targetAngle - angle;

  float control =
      (Kp * error)
    - (Kd * gyroRate)
    - (Kv * velocity);

  /* ===== Velocity Estimate ===== */

  velocity += control * dt;
  velocity *= 0.97;
  velocity = constrain(velocity, -50.0, 50.0);

  /* ===== Motor Command ===== */

  driveMotors(control);

  /* ===== Debug Output ===== */

  Serial.print("Angle=");
  Serial.print(angle, 2);
  Serial.print("  Vel=");
  Serial.print(velocity, 2);
  Serial.println();
}