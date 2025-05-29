#include <Pixy2.h>
Pixy2 pixy;

void setup() {
  Serial.begin(9600);
  pixy.init();
}

void loop() {
  int result = pixy.ccc.getBlocks();
  Serial.println(result);  // Should return ≥ 0 if working, -1 if not
  delay(500);
}
