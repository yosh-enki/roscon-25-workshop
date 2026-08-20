# ESP32 Native Servo Gripper Actuator Firmware

## 🎯 Overview
This firmware runs on an ESP32 microcontroller board to act as a **Physical Hardware-in-the-Loop (HITL) Actuator Module** for the Full Self-Driving autonomous drone delivery system.

It translates serial ASCII commands received over USB (`/dev/ttyACM0` or `/dev/ttyUSB0`) into 50Hz PWM signals controlling a physical servo motor, and returns non-blocking acknowledgment strings back to the host computer / ROS 2 bridge.

---

## 🔌 Hardware Wiring & Pinout
| ESP32 Pin | Servo Wire Color | Function | Description |
|---|---|---|---|
| **GPIO 18** | Orange / Yellow | PWM Signal | 50 Hz PWM (500 µs – 2400 µs pulse width) |
| **5V / VIN / VBUS** | Red | Power | 5V DC supply for servo motor |
| **GND** | Black / Brown | Ground | Common Ground connection |

---

## 🛠️ Arduino IDE Setup & Flashing
1. **Board Support**:
   * Open Arduino IDE $\rightarrow$ **Settings / Preferences** $\rightarrow$ Additional Board Manager URLs:
     `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   * Go to **Tools** $\rightarrow$ **Board** $\rightarrow$ **Boards Manager** $\rightarrow$ Search `esp32` by *Espressif Systems* $\rightarrow$ Install.
2. **Library**:
   * Go to **Tools** $\rightarrow$ **Manage Libraries...** $\rightarrow$ Search `ESP32Servo` by *Kevin Harrington* $\rightarrow$ Install.
3. **Flashing**:
   * Open [`esp32_gripper_actuator.ino`](./esp32_gripper_actuator.ino).
   * Select Board: `ESP32 Dev Module` (or your ESP32 model).
   * Port: `/dev/ttyACM0` (or `/dev/ttyUSB0`).
   * Baud: `115200`.
   * Click **Upload**.

---

## 📡 Serial Protocol Specification (@ 115200 baud, 8N1)
All commands are newline-terminated (`\n` or `\r\n`):

| Inbound Command | Firmware Action | Outbound Acknowledgment |
|---|---|---|
| `CMD:PING\n` | Connectivity check | `ACK:PONG\n` |
| `CMD:SECURE\n` | Moves servo to **90°** (Lock / Payload Secured) | `ACK:SECURED\n` |
| `CMD:RELEASE\n` | Moves servo to **0°** (Open / Payload Released) | `ACK:RELEASED\n` |
| *Other / Unknown* | No motion | `ERR:UNKNOWN_COMMAND\n` |

---

## 🧪 Hardware Verification (Smoke Test)
To verify your physical wiring and servo movement before running full simulations:

```bash
cd /home/yosh/roscon-25-workshop
./scripts/test_esp32_gripper.py /dev/ttyACM0
```
Expected output:
```
============================================================
  ROSCON-25 WORKSHOP: ESP32 SERVO HARDWARE SMOKE TEST
============================================================
[1/3] Testing Ping / Heartbeat...
      --> TX: 'CMD:PING\n'
      <-- RX: 'ACK:PONG' (Latency: 2.1 ms) [PASS]

[2/3] Testing Gripper SECURE (Hold / 90°)...
      --> TX: 'CMD:SECURE\n'
      <-- RX: 'ACK:SECURED' (Actuation time: 301.8 ms) [PASS]

[3/3] Testing Gripper RELEASE (Drop / 0°)...
      --> TX: 'CMD:RELEASE\n'
      <-- RX: 'ACK:RELEASED' (Actuation time: 302.4 ms) [PASS]

============================================================
  🎉 ALL HARDWARE TESTS PASSED SUCCESSFULLY!
============================================================
```

---

## 🚀 Running with HITL Drone Simulation
Launch the entire system in Docker with GPU acceleration and USB passthrough:

```bash
cd /home/yosh/roscon-25-workshop
./scripts/run_hitl_delivery.sh /dev/ttyACM0
```
Open **Foxglove Studio** at `ws://localhost:8766` (or `ws://localhost:8765`), load [`foxglove/roscon-25-workshop.json`](../../foxglove/roscon-25-workshop.json), and control the gripper in real time!
