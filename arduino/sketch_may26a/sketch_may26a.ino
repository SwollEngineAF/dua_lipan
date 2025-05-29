#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define SERVO_MIN 150   // PWM value for 0 degrees
#define SERVO_30  275   // PWM value for ~30 degrees (adjust if needed)
#define SERVO_NEUTRAL 215  // Midpoint (tweak based on your servo)

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz
  delay(500);
}

void loop() {
  testServo(0);   // Base
  testServo(4);   // Right
  testServo(11);  // Left
  testServo(15);  // Arm
  while (1);      // Stop after one full test cycle
}

void testServo(uint8_t channel) {
  pwm.setPWM(channel, 0, SERVO_NEUTRAL);
  delay(500);
  
  pwm.setPWM(channel, 0, SERVO_30);  // Move +30°
  delay(10000);                      // Hold 10 seconds
  
  pwm.setPWM(channel, 0, SERVO_NEUTRAL);  // Return to neutral
  delay(1000);
}
