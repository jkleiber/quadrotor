
#include <IBusBM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PIDController.h>
#include <SD.h>
#include <SPI.h>

#define LOG_FN_LEN 20

// IBus Interface
IBusBM ibus;

// Remote control values
int channel_values[6];

// SD card logging
const int sd_chip_select = BUILTIN_SDCARD;
char log_base_name[] = "flight_";
char log_file_name[LOG_FN_LEN];
int flight_num = 0;
String log_headers = "t,is_crashed,roll,pitch,yaw,throttle,roll_pid_out,pitch_pid_out,yaw_pid_out,motor_fl,motor_bl,motor_fr,motor_br,pwm_fl,pwm_bl,pwm_fr,pwm_br";
bool is_logging_available = false;

// Debugging
bool is_rc_debug = false;
bool is_imu_debug = true;
bool is_pid_debug = false;
bool is_scope_debug = false;

// Crash detection
bool is_crashed = false;
float roll_limit = 90;
float pitch_limit = 90;

// Scope Pin
const int scope_pin = A14;

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
const int min_pwm_duty = (int)(0.051 * (float)max_pwm);
const int max_pwm_duty = (int)(0.1 * (float)max_pwm);

// RC channels (0-indexed)
const int throttle_channel = 2; // This is Channel 3 when 1-indexed


// Throttle cutoff
// Note: This is equivalent to ~1020 us input on the RC.
const float throttle_cutoff = 0.02f; // Motor power 2%




// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(69, 0x28);

// States
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;


// Control setpoints
float roll_setpoint = 0.0;
float pitch_setpoint = 0.0;
float yaw_setpoint = 0.0;

// PID Gains
// Roll
float roll_kp = 0.1;
float roll_ki = 0.001;
float roll_kd = 0.05;
// Pitch
float pitch_kp = 0.1;
float pitch_ki = 0.001;
float pitch_kd = 0.05;
// Yaw
float yaw_kp = 0.1;
float yaw_ki = 0.001;
float yaw_kd = 0.05;

// PID output limits
// This corresponds to a change of ~312 us in the PWM pulse period.
float pid_lower = -0.2;
float pid_upper = 0.2;

// PID Controllers
PIDController roll_pid = PIDController(roll, roll_kp, roll_ki, roll_kd);
PIDController pitch_pid = PIDController(pitch, pitch_kp, pitch_ki, pitch_kd);
PIDController yaw_pid = PIDController(yaw, yaw_kp, yaw_ki, yaw_kd);

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

float deg_to_rad(float deg)
{
  return deg * PI / 180.0;
}
float rad_to_det(float rad)
{
  return rad * 180.0 / PI;
}

// Find yaw to be an angle offset relative to the yaw setpoint
// This is to maintain transitioning from the simulator until the
// PID controller is re-implemented better.
float augment_yaw(float yaw_setpoint, float yaw)
{
  // Assumptions:
  // 1. Yaw is in [0, 360)
  // 2. Yaw setpoint can be in (-inf, inf)
  // 3. Both yaw and yaw setpoint are in degrees

  // Convert yaw setpoint to be in [0, 360)
  float setpoint_0_360 = fmod(yaw_setpoint, 360);
  if (setpoint_0_360 < 0.0)
  {
    setpoint_0_360 += 360;
  }

  // Two ways of going around a circle.
  float delta_1 = 0.0;
  float delta_2 = 0.0;

  // Find the two differences if yaw setpoint is larger than yaw
  if (yaw_setpoint > yaw)
  {
    // Direct path
    delta_1 = -1.0 * (yaw_setpoint - yaw);
    // Going around the wrap-around point
    delta_2 = (360 - yaw_setpoint) + yaw;

    // flip the direction
    if(abs(delta_2) < abs(delta_1))
    {
      delta_1 = delta_2;
    }
  }
  else
  {
    // Direct path
    delta_1 = yaw - yaw_setpoint;
    // Going around the wrap-around point
    delta_2 = (360 - yaw) + yaw_setpoint;

    // flip the direction if the second way is shorter
    if(abs(delta_2) < abs(delta_1))
    {
      delta_1 = -1.0 * delta_2;
    }
  }

  // Set the output yaw to be in the neighborhood of the setpoint
  float yaw_out = yaw_setpoint + delta_1;

  // Convert to radians
  yaw_out = deg_to_rad(yaw_out);

  return yaw_out;

}


int get_flight_number()
{
  int i = 1;
  while (i <= 1000000)
  {
    // Convert the number to a string
    char buf[12];
    itoa(i, buf, 10);

    // Make the name based on the other log files
    char name[20];
    strcat(name, log_base_name);
    strcat(name, buf);

    // If the name isn't taken, then set it.
    // Also, only allow logging if a file name exists
    if(!SD.exists(name))
    {
      return i;
    }

    // Increment counter and try again.
    i++;
  }

  // Error if there is no flight number
  return -1;
}




void setup() 
{
  // Set up USB Serial for debugging
  if (is_rc_debug || is_imu_debug || is_pid_debug || is_scope_debug)
  {
    Serial.begin(115200);
  }

  if (!bno.begin())
  {
    // There was a problem detecting the BNO055.
    Serial.print("No BNO055 detected - aborting flight control");
    while (1);
  }

  // If the SD card is inserted, enable logging to the SD card.
  if (SD.begin(sd_chip_select)) 
  {
    // Name the file
    flight_num = get_flight_number();
    

    if (flight_num >= 0)
    {
      // Logging is available
      is_logging_available = true;

      // Stringify the flight number
      char flight_str_buf[12];
      itoa(flight_num, flight_str_buf, 10); // 10 = base-10

      // Form the log filename
      strcat(log_file_name, log_base_name);
      strcat(log_file_name, flight_str_buf);

      // const char* version of the log filename
      const char* logfile = const_cast<const char *>(log_file_name);
      
      // Set logging headers
      File log_file = SD.open(logfile, FILE_WRITE);
      log_file.println(log_headers);
      log_file.close();
    }
  }
  
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

  // Set PID output limits
  roll_pid.setOutputRange(pid_lower, pid_upper);
  pitch_pid.setOutputRange(pid_lower, pid_upper);
  yaw_pid.setOutputRange(pid_lower, pid_upper);

  // Wait for BNO055 to initialize
  delay(5000);
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

  // Get a sensor event from the BNO055 for orientation.
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

  // Set the roll, pitch and yaw.
  // Note: this depends on the orientation of the BNO055 in the quadcopter itself.
  // Modify as needed to achieve the correct orientation.
  roll = -1.0*event.orientation.z;
  pitch = -1.0*event.orientation.y;
  yaw = event.orientation.x;

  // Compute yaw as the closest angle to the yaw setpoint
  // This also converts yaw to radians
  yaw = augment_yaw(yaw_setpoint, yaw);

  // Convert roll and pitch to radians
  roll = deg_to_rad(roll);
  pitch = deg_to_rad(pitch);

  if (is_imu_debug)
  {
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Yaw: ");
    Serial.println(yaw);
  }

  // Convert the rc throttle input to a motor power percentage
  float throttle = convert_rc_throttle_to_power(channel_values[throttle_channel]);

  // The quadcopter is crashed if it rolls or pitches too much.
  if (fabs(roll) >= roll_limit || fabs(pitch) >= pitch_limit)
  {
    is_crashed = true;
  }

  // The quadcopter can only be un-crashed once the throttle is set low and the quadcopter is upright
  if (throttle < throttle_cutoff && fabs(roll) < roll_limit && fabs(pitch) < pitch_limit)
  {
    is_crashed = false;
  }

  // PID control calculations
  float roll_output = roll_pid.update(roll_setpoint, roll);
  float pitch_output = pitch_pid.update(pitch_setpoint, pitch);
  float yaw_output = yaw_pid.update(yaw_setpoint, yaw);

  // Don't do anything unless the throttle is outside a deadband.
  if (throttle >= throttle_cutoff && !is_crashed)
  {
    // Compute power for each motor.
    front_left_out  = throttle - roll_output + pitch_output + yaw_output; 
    back_left_out   = throttle - roll_output - pitch_output - yaw_output;
    front_right_out = throttle + roll_output + pitch_output - yaw_output; 
    back_right_out  = throttle + roll_output - pitch_output + yaw_output; 

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

    if (is_pid_debug)
    {
      Serial.print("FL: ");
      Serial.print(front_left_out);
      Serial.print(" BL: ");
      Serial.print(back_left_out);
      Serial.print(" FR: ");
      Serial.print(front_right_out);
      Serial.print(" BR: ");
      Serial.println(back_right_out);
    }
  }
  // Otherwise output the idle (motors off) duty cycle.
  else
  {
    front_left_out = 0.0;
    back_left_out = 0.0;
    front_right_out = 0.0;
    back_right_out = 0.0;

    pwm_fl = idle_out;
    pwm_bl = idle_out;
    pwm_fr = idle_out;
    pwm_br = idle_out;

    // Reset the PIDs
    roll_pid.reset();
    pitch_pid.reset();
    yaw_pid.reset();
  }

  // Motor outputs
  analogWrite(front_left_motor, pwm_fl);
  analogWrite(back_left_motor, pwm_bl);
  analogWrite(front_right_motor, pwm_fr);
  analogWrite(back_right_motor, pwm_br);

  if (is_scope_debug)
  {
    int scope_input = analogRead(scope_pin);
    Serial.println(scope_input);
  }

  // Logging
  if (is_logging_available)
  {
    // const char* version of the log filename
      const char* logfile = const_cast<const char *>(log_file_name);

    // Create a data string
    String data = String(millis()) + "," + String(is_crashed) + "," + String(roll) + "," + String(pitch) + "," + String(yaw);
    data += "," + String(throttle) + "," + String(roll_output) + "," + String(pitch_output) + "," + String(yaw_output);
    data += "," + String(front_left_out) + "," + String(back_left_out) + "," + String(front_right_out) + "," + String(back_right_out);
    data += "," + String(pwm_fl) + "," + String(pwm_bl) + "," + String(pwm_fr) + "," + String(pwm_br);

    // Open the file, write a new line and then close.
    File log_file = SD.open(logfile, FILE_WRITE);
    log_file.println(data);
    log_file.close();
  }
  

  // Run no faster than 1000 Hz
  delay(1);
}
