 Self-Balancing Robot

A two-wheeled self-balancing robot using an ESP32, MPU6050 IMU, and DRV8833 motor driver.  
Maintains upright balance in real time using a complementary filter and PD control.

## Features

- Real-time tilt estimation (gyro + accelerometer)
- PD control with velocity damping
- PWM deadzone and slew rate compensation
- Safety cutoff on large tilts

## Hardware

- ESP32, MPU6050, DRV8833, 2 DC motors
- Li-ion battery, buck converter, 3D-printed chassis

## Demo

YouTube: [https://youtu.be/auV84QBynF0](https://youtu.be/auV84QBynF0)

## Author

Máximo Mancilla
