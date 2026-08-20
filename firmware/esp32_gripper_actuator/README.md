# ESP32 Native Servo Gripper Actuator Firmware

## Overview
This firmware runs on an ESP32 microcontroller board to act as a **Physical Hardware-in-the-Loop (HITL) Actuator Module** for the Full Self-Driving autonomous drone delivery system.

## Hardware Wiring
| ESP32 Pin | Servo Wire | Description |
|---|---|---|
| **GPIO 18** | Orange / Yellow | PWM Signal (50Hz, 500µs - 2400µs) |
| **5V / VIN / VBUS** | Red | Servo Power (5V) |
| **GND** | Black / Brown | Common Ground |

## Prerequisites (Arduino IDE)
1. Install **ESP32 Board Support** in Arduino IDE (`Boards Manager` -> search `esp32` by Espressif Systems).
2. Install **ESP32Servo** library (`Library Manager` -> search `ESP32Servo` by Kevin Harrington).

## Flashing Instructions
1. Connect ESP32 via USB cable (`/dev/ttyACM0` or `/dev/ttyUSB0`).
2. Open `firmware/esp32_gripper_actuator/esp32_gripper_actuator.ino` in Arduino IDE or VSCode / PlatformIO.
3. Select Board: `ESP32 Dev Module` (or your specific ESP32 variant).
4. Port: `/dev/ttyACM0`.
5. Baud rate: `115200`.
6. Click **Upload**.

## Serial Protocol Specification (@ 115200 baud)
- `CMD:PING\n` -> Replies `ACK:PONG\n` (Heartbeat / connectivity test)
- `CMD:SECURE\n` -> Moves servo to 0° (Locked position) -> Replies `ACK:SECURED\n`
- `CMD:RELEASE\n` -> Moves servo to 180° (Drop / open position) -> Replies `ACK:RELEASED\n`
