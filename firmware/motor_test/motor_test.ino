
#include <IBusBM.h>

// IBus interface
IBusBM ibus;

// ESC calibration pin
const int esc_calib_pin = 23;

// Check PWM
const int pwm_analog_pin = A5;

// Throttle channel
const int throttle_channel = 2; // This is Channel 3 when 1-indexed

void setup() {
  // Set up the IBusBM Serial interface.
  // Use IBUSBM_NOTIMER to run the loop ourselves.
  // Timing in IBusBM is not supported on Teensy 3.6.
  ibus.begin(Serial4, IBUSBM_NOTIMER);

  // Set up USB serial
  Serial.begin(115200);

  // Set up the ESC calibration pin
  pinMode(esc_calib_pin, OUTPUT);

  // We want 50Hz PWM for controlling the ESC.
  analogWriteFrequency(esc_calib_pin, 50);

  // Set the resolution to 8 bits (0-255).
  analogWriteResolution(8);
}

void loop() {
  // Run the IBusBM loop here since the interrupt 
  // timing is not supported on Teensy 3.6.
  ibus.loop();

  // Read the throttle channel
  int throttle = ibus.readChannel(throttle_channel);
  if(throttle < 1000){
    throttle = 1000;
  }

  // Convert the throttle into a duty cycle
  float duty_cycle = ((float)throttle / 20000) * 255.0;

  // Apply the output to the ESC
  int pwm_duty = (int)duty_cycle;
  analogWrite(esc_calib_pin, pwm_duty);

  // Read the PWM output
  int pwm_out = analogRead(pwm_analog_pin);

//  // Print the throttle and duty cycle commanded
//  Serial.print("Throttle: ");
//  Serial.print(throttle);
//  Serial.print(" Duty: ");
//  Serial.println(pwm_duty);

  // Print the PWM waveform to the Serial Plotter
  Serial.print("PWM: ");
  Serial.println(pwm_out);
}
