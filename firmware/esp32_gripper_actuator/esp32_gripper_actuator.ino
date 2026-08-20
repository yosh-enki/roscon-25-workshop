#include <ESP32Servo.h>

Servo myServo;

// Pin configuration
const int SERVO_PIN = 18;       // Signal pin connected to Servo PWM wire (Orange/Yellow)
const int ANGLE_SECURED = 0;    // Locked/Secured for flight (0 degrees)
const int ANGLE_RELEASED = 180; // Open/Released for cargo drop (180 degrees)
const int DWELL_TIME_MS = 300;  // Mechanical transit settling time

void setup() {
  Serial.begin(115200);
  
  // Allocate ESP32 PWM hardware timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  myServo.setPeriodHertz(50);             // Standard 50Hz PWM
  myServo.attach(SERVO_PIN, 500, 2400);   // 500us (0 deg) to 2400us (180 deg)
  myServo.write(ANGLE_SECURED);           // Default to locked position on boot
  
  Serial.println("ESP32_GRIPPER_READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "CMD:RELEASE") {
      myServo.write(ANGLE_RELEASED);
      delay(DWELL_TIME_MS);
      Serial.println("ACK:RELEASED");
    } else if (cmd == "CMD:SECURE") {
      myServo.write(ANGLE_SECURED);
      delay(DWELL_TIME_MS);
      Serial.println("ACK:SECURED");
    } else if (cmd == "CMD:PING") {
      Serial.println("ACK:PONG");
    } else if (cmd.length() > 0) {
      Serial.print("ERR:UNKNOWN_CMD:");
      Serial.println(cmd);
    }
  }
}
