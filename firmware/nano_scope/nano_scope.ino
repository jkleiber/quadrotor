
const int scope_pin = 2;

const int trigger = 500; // square wave trigger
volatile unsigned long last_low = 0;
volatile unsigned long last_high = 0;
bool is_high = false;
float frequency = 0;

volatile int down_time = 0;
volatile int up_time = 0;

volatile int signal_val = 1;

void scope_trigger()
{
  signal_val = digitalRead(scope_pin);

  if(signal_val == HIGH)
  {
    down_time = millis() - last_high;
    last_low = millis();
  }
  else
  {
    up_time = millis() - last_low;
    last_high = millis();
  }
}



void setup() {
  // put your setup code here, to run once:
  pinMode(scope_pin, INPUT_PULLUP);

  Serial.begin(115200);

  // Frequency trackers
  last_low = millis();
  last_high = millis();

  attachInterrupt(digitalPinToInterrupt(scope_pin), scope_trigger, CHANGE);
  
}

void loop() {
  Serial.print("Uptime:");
  Serial.print(up_time);
  Serial.print(",");
  Serial.print("DownTime:");
  Serial.print(down_time);
  Serial.print(",");
  Serial.print("Signal:");
  Serial.println(signal_val);
//  int val = 100*digitalRead(scope_pin);
//  
//
//  // Track frequency
//  if (val == HIGH && !is_high)
//  {
//    // Compute frequency
//    frequency = 1.0 / ((float)(millis() - last_low) / 1000.0);
//    
//    is_high = true;
//    last_low = millis();
//  }
//  else if(val == LOW && is_high)
//  {
//    // Compute frequency
////    frequency = 1.0 / ((float)(millis() - last_high) / 1000.0);
//
//    is_high = false;
//    last_high = millis();
//  }
//
//  Serial.print("Signal:");
//  Serial.print(val);
//  Serial.print(",");
//  Serial.print("Freq:");
//  Serial.println(frequency);
  
  delay(1);
}
