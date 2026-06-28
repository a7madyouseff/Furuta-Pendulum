// ============================================
// FURUTA PENDULUM - LQR Balancing Controller
// Motor encoder (alpha) + AS5600 (beta)
// Arduino Uno
// ============================================

#include <Wire.h>

// ----- Motor Driver Pins -----
#define IN1 7
#define IN2 8
#define ENA 9

// ----- Motor Encoder Pins (arm angle alpha) -----
#define MOTOR_ENC_A 2
#define MOTOR_ENC_B 3

// ----- AS5600 Magnetic Encoder (pendulum angle beta) -----
#define AS5600_ADDR 0x36
#define RAW_ANGLE_H 0x0C

// ----- LQR Gains (from MATLAB) -----
// u = -K*x  where x = [alpha, beta, alpha_dot, beta_dot]
float K1 = -1;    // alpha gain
float K2 = -33.2177;   // beta gain (most important)
float K3 = -1.4179;    // alpha_dot gain
float K4 = -6.8072;    // beta_dot gain

// ----- Variables -----
volatile long motorCount = 0;
int MOTOR_PPR = 360;

float alpha = 0, beta = 0;
float alpha_prev = 0, beta_prev = 0;
float alpha_dot = 0, beta_dot = 0;

float beta_offset = 0;  // calibration offset
unsigned long lastTime = 0;
float dt = 0.01;  // 10ms loop

// ----- AS5600 read function -----
float readBeta() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_H);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  int highByte = Wire.read();
  int lowByte = Wire.read();
  int rawAngle = (highByte << 8) | lowByte;
  float angleDeg = (rawAngle / 4096.0) * 360.0;
  return angleDeg;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  // Motor encoder
  pinMode(MOTOR_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENC_A), readMotorEncoder, RISING);
  
  // Calibrate beta to upright = 0
  delay(500);
  beta_offset = readBeta();
  
  lastTime = millis();
}

void loop() {
  // Timing
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  if (dt < 0.01) return;  // run at 100Hz
  lastTime = now;
  
  // ----- Read states -----
  alpha = (motorCount / (float)MOTOR_PPR) * 360.0; // degrees
  beta = readBeta() - beta_offset;                  // degrees from upright
  
  // Convert to radians for control
  float alpha_rad = alpha * PI / 180.0;
  float beta_rad = beta * PI / 180.0;
  
  // ----- Compute velocities -----
  alpha_dot = (alpha_rad - alpha_prev) / dt;
  beta_dot = (beta_rad - beta_prev) / dt;
  alpha_prev = alpha_rad;
  beta_prev = beta_rad;
  
  // ----- LQR Control Law: u = -Kx -----
  float u = -(K1*alpha_rad + K2*beta_rad + K3*alpha_dot + K4*beta_dot);
  
  // ----- Apply control to motor -----
  applyMotor(u);
  
  // ----- Send data to Serial -----
  Serial.print(now/1000.0);
  Serial.print(",");
  Serial.print(alpha);
  Serial.print(",");
  Serial.println(beta);
}

void applyMotor(float u) {
  // Convert control signal to PWM
  int pwm = (int)abs(u);
  if (pwm > 255) pwm = 255;  // saturate
  
  // Safety: if pendulum fell too far, stop motor
  if (abs(beta) > 45) {
    analogWrite(ENA, 0);
    return;
  }
  
  // Set direction based on sign of u
  if (u > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwm);
}

void readMotorEncoder() {
  if (digitalRead(MOTOR_ENC_B) == HIGH) {
    motorCount++;
  } else {
    motorCount--;
  }
}
