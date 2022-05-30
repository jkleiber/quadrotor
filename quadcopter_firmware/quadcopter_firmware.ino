
#include <IBusBM.h>

// IBus Interface
IBusBM ibus;

// Remote control values
int channel_values[6];

// Debugging
bool is_debug = true;



void setup() {
  // Set up USB Serial for debugging
  if (is_debug)
  {
    Serial.begin(115200);
  }
  
  // Set up the IBusBM Serial interface.
  // Use IBUSBM_NOTIMER to run the loop ourselves.
  // Timing in IBusBM is not supported on Teensy 3.6.
  ibus.begin(Serial4, IBUSBM_NOTIMER);
}

void loop() {

  // Run the IBusBM loop here since the interrupt 
  // timing is not supported on Teensy 3.6.
  ibus.loop();

  // Read remote control channels
  for(int i = 0; i < 6; i++)
  {
    channel_values[i] = ibus.readChannel(i);

    if (is_debug)
    {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(channel_values[i]);
      Serial.print(" ");
    }
  }

  // Print a new line for debugging.
  if(is_debug)
  {
    Serial.println();
  }

  // Run approximately 200Hz
  delay(5);
}
