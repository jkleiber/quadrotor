
#include <IBusBM.h>
#include <Wire.h>
#include <SPI.h>

#define LOG_FN_LEN 20

float deg_to_rad(float deg)
{
  return deg * PI / 180.0;
}
float rad_to_det(float rad)
{
  return rad * 180.0 / PI;
}

// IBus Interface
IBusBM ibus;

// Remote control values
int channel_values[6];

// Detect that the throttle stick has been high before
bool throttle_has_been_high = false;
float min_high_throttle = 0.98f;

// Debugging
bool is_rc_debug = false;
bool is_pwm_debug = false;


// Motor PWM pins
const int front_left_motor = 23;
const int back_left_motor = 22;
const int front_right_motor = 21;
const int back_right_motor = 20;

// PWM settings
const int pwm_resolution = 14;
const int max_pwm = pow(2,pwm_resolution) - 1;

// Duty cycle conversions
const int pwm_period_us = 20000;
const int pwm_max_duty_us = 2000;
const int pwm_min_duty_us = 1000;

// Duty cycle limits
const int idle_out = max_pwm * pwm_min_duty_us / pwm_period_us;
const int calib_out = max_pwm * pwm_max_duty_us / pwm_period_us;
const int min_pwm_duty = (int)(0.051 * (float)max_pwm);
const int max_pwm_duty = (int)(0.1 * (float)max_pwm);

// RC channels (0-indexed)
const int throttle_channel = 2; // This is Channel 3 when 1-indexed


// Throttle cutoff
// Note: This is equivalent to ~1020 us input on the RC.
const float throttle_cutoff = 0.02f; // Motor power 2%



// Motor power
float front_left_out  = 0.0;
float back_left_out   = 0.0;
float front_right_out = 0.0;
float back_right_out  = 0.0;

// Motor PWMs
int pwm_fl = 0;
int pwm_bl = 0;
int pwm_fr = 0;
int pwm_br = 0;

// Convert RC throttle to percent power.
float convert_rc_throttle_to_power(int throttle)
{
  // Get the period range
  int pwm_duty_range_us = pwm_max_duty_us - pwm_min_duty_us;
  
  // Convert the throttle into percentage of the maximum control duty cycle
  float power = ((float) throttle - (float) pwm_min_duty_us) / (float)pwm_duty_range_us;

  return power;
}

// Convert the percent power for a motor to PWM duty cycle.
int convert_power_to_pwm(float power)
{
  // Get the period range
  int pwm_duty_range_us = pwm_max_duty_us - pwm_min_duty_us;
  
  // Power is a percentage of the maximum duty cycle period divided by the PWM period
  float pwm_high_us = (power * (float)pwm_duty_range_us) + (float)pwm_min_duty_us;
  float duty_cycle_pct = pwm_high_us / (float)pwm_period_us;

  // The percentage is multiplied by the PWM resolution to get the analogWrite value.
  float duty_cycle = duty_cycle_pct * (float)max_pwm;

  // Convert duty cycle to int for PWM output
  int pwm_duty = (int) duty_cycle;

  return pwm_duty;
}

int clamp_pwm_duty(int duty, int min_duty, int max_duty)
{
  duty = min(max_duty, max(min_duty, duty));

  return duty;
}




void setup() 
{
  // Set up USB Serial for debugging
  Serial.begin(115200);
  
  // Set up the IBusBM Serial interface.
  // Use IBUSBM_NOTIMER to run the loop ourselves.
  // Timing in IBusBM is not supported on Teensy 3.6.
  ibus.begin(Serial4, IBUSBM_NOTIMER);

  // Configure the motor ports to be outputs.
  pinMode(front_left_motor, OUTPUT);
  pinMode(back_left_motor, OUTPUT);
  pinMode(front_right_motor, OUTPUT);
  pinMode(back_right_motor, OUTPUT);

  // Set the PWM resolution
  analogWriteResolution(pwm_resolution);

  // Set the PWM frequencies for each motor to 50 Hz.
  analogWriteFrequency(front_left_motor, 50);
  analogWriteFrequency(back_left_motor, 50);
  analogWriteFrequency(front_right_motor, 50);
  analogWriteFrequency(back_right_motor, 50);

  // Set ESCs to max output per calibration instructions.
  analogWrite(front_left_motor, calib_out);
  analogWrite(back_left_motor, calib_out);
  analogWrite(front_right_motor, calib_out);
  analogWrite(back_right_motor, calib_out);
}

void loop() {

  // Run the IBusBM loop here since the interrupt 
  // timing is not supported on Teensy 3.6.
  ibus.loop();

  // Read remote control channels
  for(int i = 0; i < 6; i++)
  {
    channel_values[i] = ibus.readChannel(i);

    if (is_rc_debug)
    {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(channel_values[i]);
      Serial.print(" ");
    }
  }

  // Print a new line for debugging.
  if(is_rc_debug)
  {
    Serial.println();
  }

  // Convert the rc throttle input to a motor power percentage
  float throttle = convert_rc_throttle_to_power(channel_values[throttle_channel]);

  if (throttle > min_high_throttle) {
    throttle_has_been_high = true;
  }

  // Passthrough throttle as power for each motor.
  front_left_out  = throttle; 
  back_left_out   = throttle;
  front_right_out = throttle; 
  back_right_out  = throttle; 

  // Convert power to PWM
  pwm_fl = convert_power_to_pwm(front_left_out);
  pwm_bl = convert_power_to_pwm(back_left_out);
  pwm_fr = convert_power_to_pwm(front_right_out);
  pwm_br = convert_power_to_pwm(back_right_out);

  // Clamp the duty cycles
  pwm_fl = clamp_pwm_duty(pwm_fl, min_pwm_duty, max_pwm_duty);
  pwm_bl = clamp_pwm_duty(pwm_bl, min_pwm_duty, max_pwm_duty);
  pwm_fr = clamp_pwm_duty(pwm_fr, min_pwm_duty, max_pwm_duty);
  pwm_br = clamp_pwm_duty(pwm_br, min_pwm_duty, max_pwm_duty);

  if (is_pwm_debug) {
      Serial.print("FL: ");
      Serial.print(pwm_fl);
      Serial.print(" BL: ");
      Serial.print(pwm_bl);
      Serial.print(" FR: ");
      Serial.print(pwm_fr);
      Serial.print(" BR: ");
      Serial.println(pwm_br);
  }

  // Only output new values once the RC throttle stick has been high before.
  if (throttle_has_been_high) {
    // Motor outputs
    analogWrite(front_left_motor, pwm_fl);
    analogWrite(back_left_motor, pwm_bl);
    analogWrite(front_right_motor, pwm_fr);
    analogWrite(back_right_motor, pwm_br);
  }
  
  // Run no faster than 1000 Hz
  delay(1);
}
