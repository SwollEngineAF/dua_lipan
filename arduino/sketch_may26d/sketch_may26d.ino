#include <Pixy2.h>
Pixy2 pixy;

void setup() {
  Serial.begin(9600);
  pixy.init();
}

void loop() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0) {
    Serial.print(pixy.ccc.blocks[0].m_x);
    Serial.print(",");
    Serial.println(pixy.ccc.blocks[0].m_y);
  }
  delay(100);
}
