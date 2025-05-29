#include <Arduino.h>

void setup() {
  pinMode(13, OUTPUT); // Built-in LED
  pinMode(2, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  digitalWrite(2, LOW);
  delay(1000);
}
