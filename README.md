# Furuta-Pendulum
# Fixed-Pivot Inverted Pendulum Control System

This project implements a fixed-pivot inverted pendulum system using an Arduino-based control setup. The goal is to balance a freely rotating pendulum in the upright position by controlling the motion of a DC motor arm.

The system uses a DC motor with an encoder to measure the arm position and an AS5600 magnetic encoder to measure the pendulum angle. A feedback controller is implemented on an Arduino to stabilize the pendulum around the upright equilibrium point.

## Project Overview

The inverted pendulum is a classic unstable control system. Without control, the pendulum naturally falls due to gravity. In this project, the controller continuously measures the pendulum angle and moves the motor arm to correct the pendulum motion and keep it balanced.

The project includes:

- Mechanical design of the pendulum and motor arm
- Sensor integration using motor encoder and AS5600 magnetic encoder
- Arduino-based real-time control
- Motor driving using the L298N motor driver
- Controller tuning and testing
- MATLAB analysis of system response

## Hardware Used

- Arduino UNO
- DC motor with built-in encoder
- L298N motor driver
- AS5600 magnetic angle sensor
- 12 V external motor power supply
- 5 V logic supply
- 3D-printed motor arm and pendulum parts
- Bearing-supported pendulum pivot
- Permanent magnet for AS5600 sensing

## Wiring Summary

### Motor Driver L298N

| L298N Pin | Arduino / Supply |
|----------|------------------|
| IN1 | D7 |
| IN2 | D8 |
| ENA | D9 PWM |
| GND | Common GND |
| VCC | 12 V motor supply |
| 5V | 5 V logic |
| OUT1 / OUT2 | DC motor terminals |

### Motor Encoder

| Encoder Pin | Arduino |
|------------|---------|
| VCC | 5 V |
| GND | GND |
| Channel A | D2 interrupt |
| Channel B | D4 |

### AS5600 Magnetic Encoder

| AS5600 Pin | Arduino |
|-----------|---------|
| VCC | 5 V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

## Control Method

The controller is implemented in Arduino code using feedback from the pendulum angle. The best experimental performance was achieved using a PD controller.

The controller calculates the pendulum angle error and angular velocity, then generates a PWM signal for the motor driver. The motor direction is adjusted depending on the required correction direction.

Main control parameters:

```cpp
float Kp = 11.0;
float Kd = 1.3;
float Ki = 0.0;

int minPWM = 40;
int maxPWM = 230;

float deadband = 1.2;
float activeAngleLimit = 30.0;
int MOTOR_SIGN = -1;
