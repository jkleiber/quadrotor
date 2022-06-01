
#include <IBusBM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PIDController.h>


// IBus Interface
IBusBM ibus;

// Remote control values
int channel_values[6];

// Debugging
bool is_rc_debug = false;
bool is_imu_debug = true;
bool is_pid_debug = false;
bool is_scope_debug = false;

// Scope Pin
const int scope_pin = A14;

// Motor PWM
const int front_left_motor = 23;
const int back_left_motor = 22;
const int front_right_motor = 21;
const int back_right_motor = 20;

// Duty cycle limits
const int idle_out = 12;
const int min_pwm_duty = 13;
const int max_pwm_duty = 25;

// RC channels (0-indexed)
const int throttle_channel = 2; // This is Channel 3 when 1-indexed


// Throttle cutoff
// Note: This is equivalent to ~1019 us.
const int throttle_cutoff = 13; // PWM duty cycle

// Duty cycle conversions
const int pwm_period_us = 20000;


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
float roll_kp = 0.8;
float roll_ki = 0.0;
float roll_kd = 1.0;
// Pitch
float pitch_kp = 0.8;
float pitch_ki = 0.0;
float pitch_kd = 1.0;
// Yaw
float yaw_kp = 1.0;
float yaw_ki = 0.0;
float yaw_kd = 0.0;

// PID output limits
// This corresponds to a change of ~312 us in the PWM pulse period.
float pid_lower = -4.0;
float pid_upper = 4.0;

// PID Controllers
PIDController roll_pid = PIDController(roll, roll_kp, roll_ki, roll_kd);
PIDController pitch_pid = PIDController(pitch, pitch_kp, pitch_ki, pitch_kd);
PIDController yaw_pid = PIDController(yaw, yaw_kp, yaw_ki, yaw_kd);

// Convert throttle commands into duty cycle output for the motors.
int convert_throttle_to_pwm(int throttle)
{
  // Convert the throttle into a duty cycle
  float duty_cycle = ((float)throttle / pwm_period_us) * 255.0;

  // Apply the output to the ESC
  int pwm_duty = (int)duty_cycle;

  return pwm_duty;
}

int clamp_pwm_duty(int duty, int min_duty, int max_duty)
{
  duty = min(max_duty, max(min_duty, duty));

  return duty;
}


void setup() {
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
  
  // Set up the IBusBM Serial interface.
  // Use IBUSBM_NOTIMER to run the loop ourselves.
  // Timing in IBusBM is not supported on Teensy 3.6.
  ibus.begin(Serial4, IBUSBM_NOTIMER);

  // Configure the motor ports to be outputs.
  pinMode(front_left_motor, OUTPUT);
  pinMode(back_left_motor, OUTPUT);
  pinMode(front_right_motor, OUTPUT);
  pinMode(back_right_motor, OUTPUT);

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

  if (is_imu_debug)
  {
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Yaw: ");
    Serial.println(yaw);
  }

  // Compute base output using throttle
  int throttle = convert_throttle_to_pwm(channel_values[throttle_channel]);

  // Don't do anything unless the throttle is outside a deadband.
  if (throttle >= throttle_cutoff)
  {
    // PID control calculations
    float roll_output = roll_pid.update(roll_setpoint, roll);
    float pitch_output = pitch_pid.update(pitch_setpoint, pitch);
    float yaw_output = yaw_pid.update(yaw_setpoint, yaw);

    // TODO: yaw control. Currently need to contend with the angle wrapping by the BNO.
  
    // Compute effective duty cycle for each motor.
    int front_left_out  = (int)((float)throttle - roll_output + pitch_output); // ? yaw
    int back_left_out   = (int)((float)throttle - roll_output - pitch_output); // ? yaw
    int front_right_out = (int)((float)throttle + roll_output + pitch_output); // ? yaw
    int back_right_out  = (int)((float)throttle + roll_output - pitch_output); // ? yaw

    // Clamp the duty cycles
    front_left_out  = clamp_pwm_duty(front_left_out, min_pwm_duty, max_pwm_duty);
    back_left_out   = clamp_pwm_duty(back_left_out, min_pwm_duty, max_pwm_duty);
    front_right_out = clamp_pwm_duty(front_right_out, min_pwm_duty, max_pwm_duty);
    back_right_out  = clamp_pwm_duty(back_right_out, min_pwm_duty, max_pwm_duty);

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

    // Motor outputs
    analogWrite(front_left_motor, front_left_out);
    analogWrite(back_left_motor, back_left_out);
    analogWrite(front_right_motor, front_right_out);
    analogWrite(back_right_motor, back_right_out);
  }
  // Otherwise output the idle (motors off) duty cycle.
  else
  {
    analogWrite(front_left_motor, idle_out);
    analogWrite(back_left_motor, idle_out);
    analogWrite(front_right_motor, idle_out);
    analogWrite(back_right_motor, idle_out);
  }

  if (is_scope_debug)
  {
    int scope_input = analogRead(scope_pin);
//    Serial.print("Scope:");
    Serial.println(scope_input);
  }
  

  // Run no faster than 1000 Hz
  delay(1);
}
