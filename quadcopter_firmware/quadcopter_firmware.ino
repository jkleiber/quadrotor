
#include <IBusBM.h>

IBusBM ibus;
char buf[256];

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  ibus.begin(Serial4, IBUSBM_NOTIMER);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = 0;

  ibus.loop();

  for(int i = 0; i < 6; i++)
  {
    val = ibus.readChannel(i);
    Serial.print(i);
    Serial.print(": ");
    Serial.print(val);
    Serial.print(" ");
  }
  Serial.println();


  delay(20);
}
