#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    int ch = Serial.read();        // Channel (0–15)
    int val = Serial.parseInt();   // PWM value
    pwm.setPWM(ch, 0, val);
  }
}
