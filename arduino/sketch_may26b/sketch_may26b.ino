#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Safe PWM ranges per servo
#define BASE_MIN     150
#define BASE_MAX     550

#define RIGHT_MIN    350
#define RIGHT_MAX    550

#define LEFT_MIN     380
#define LEFT_MAX     550

#define ARM_MIN      300
#define ARM_MAX      450

// Sweep speed and step size
#define STEP         5
#define DELAY_MS     100

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz for standard servos
  delay(1000);
}

void loop() {
  // Sweep forward
  for (int pos = 0; pos <= 100; pos += STEP) {
    setServoRange(pos / 100.0);
    delay(DELAY_MS);
  }

  // Sweep backward
  for (int pos = 100; pos >= 0; pos -= STEP) {
    setServoRange(pos / 100.0);
    delay(DELAY_MS);
  }
}

// Map a normalized position (0.0 to 1.0) to each servo's min-max range
void setServoRange(float percent) {
  pwm.setPWM(0, 0, mapPercent(BASE_MIN, BASE_MAX, percent));   // Base
  pwm.setPWM(4, 0, mapPercent(RIGHT_MIN, RIGHT_MAX, percent)); // Right
  pwm.setPWM(11, 0, mapPercent(LEFT_MIN, LEFT_MAX, percent));  // Left
  pwm.setPWM(15, 0, mapPercent(ARM_MIN, ARM_MAX, percent));    // Arm
}

// Helper function to map 0.0–1.0 to servo PWM value
int mapPercent(int minVal, int maxVal, float percent) {
  return minVal + (int)((maxVal - minVal) * percent);
}
