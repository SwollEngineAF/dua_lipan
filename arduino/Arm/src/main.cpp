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

#define GRIP_OPEN    450  // PWM to open claw (was ARM_MIN)
#define GRIP_CLOSE   300  // PWM to close claw (was ARM_MAX)

// --- Function Prototype ---
void setServoPose(int base, int right, int left, int grip);

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(1000);

  Serial.println("🟢 Robot arm ready. Running initial sequence...");

  Serial.println("→ Moving to grab pose...");
  setServoPose(500, 550, 550, GRIP_OPEN);  // move all joints and open grip
  delay(3000);

  Serial.println("→ Grabbing object...");
  pwm.setPWM(15, 0, GRIP_CLOSE);  // close gripper
  delay(1500);

  Serial.println("→ Lifting (static)...");
  setServoPose(500, 350, 380, GRIP_CLOSE);  // simulate "lift"
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("B:")) {
      int val = input.substring(2).toInt();
      pwm.setPWM(0, 0, constrain(val, BASE_MIN, BASE_MAX));
      Serial.print("📡 Base → PWM: "); Serial.println(val);
    }
    else if (input.startsWith("G:")) {
      int state = input.substring(2).toInt();
      int gripVal = (state == 1) ? GRIP_CLOSE : GRIP_OPEN;
      pwm.setPWM(15, 0, gripVal);
      Serial.print("🤖 Gripper → "); Serial.println(state == 1 ? "CLOSE" : "OPEN");
    }
    else if (input.startsWith("R:")) {
      int val = input.substring(2).toInt();
      pwm.setPWM(4, 0, constrain(val, RIGHT_MIN, RIGHT_MAX));
      Serial.print("🔧 Right Arm → PWM: "); Serial.println(val);
    }
    else if (input.startsWith("L:")) {
      int val = input.substring(2).toInt();
      pwm.setPWM(11, 0, constrain(val, LEFT_MIN, LEFT_MAX));
      Serial.print("🔧 Left Arm → PWM: "); Serial.println(val);
    }
  }
}

void setServoPose(int base, int right, int left, int grip) {
  base = constrain(base, BASE_MIN, BASE_MAX);
  right = constrain(right, RIGHT_MIN, RIGHT_MAX);
  left = constrain(left, LEFT_MIN, LEFT_MAX);
  grip = constrain(grip, GRIP_CLOSE, GRIP_OPEN);

  pwm.setPWM(0, 0, base);   // Base
  pwm.setPWM(4, 0, right);  // Right arm
  pwm.setPWM(11, 0, left);  // Left arm
  pwm.setPWM(15, 0, grip);  // Gripper

  Serial.print("🚀 Pose → B:");
  Serial.print(base);
  Serial.print(" R:");
  Serial.print(right);
  Serial.print(" L:");
  Serial.print(left);
  Serial.print(" G:");
  Serial.println(grip);
}
