
#include "lpf.h"
#include "pid.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <IBusBM.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <utility/imumaths.h>

#define LOG_FN_LEN 20

float deg_to_rad(float deg) { return deg * PI / 180.0; }
float rad_to_det(float rad) { return rad * 180.0 / PI; }

// IBus Interface
IBusBM ibus;

// Remote control values
int channel_values[6];

// SD card logging
const int sd_chip_select = BUILTIN_SDCARD;
char log_base_name[] = "flight_\0";
char log_file_name[LOG_FN_LEN] = {'\0'};
int flight_num = 0;
String log_headers =
    "t,is_crashed,roll,pitch,yaw,p,q,r,throttle,p_cmd,q_cmd,r_cmd,roll_cmd,"
    "pitch_cmd,yaw_cmd,roll_pid_out,pitch_pid_out,yaw_pid_out,motor_fl,motor_"
    "bl,motor_fr,motor_br,pwm_fl,pwm_bl,pwm_fr,pwm_br";
bool is_logging_available = false;
const int kLogPrecision = 6;

// Debugging
bool is_rc_debug = false;
bool is_imu_debug = false;
bool is_pid_debug = false;
bool is_scope_debug = false;
bool is_pwm_debug = false;

// Crash detection
bool is_crashed = false;
float roll_limit = deg_to_rad(90);
float pitch_limit = deg_to_rad(90);

// Scope Pin
const int scope_pin = A14;

// Motor PWM pins
const int front_left_motor = 23;
const int back_left_motor = 22;
const int front_right_motor = 21;
const int back_right_motor = 20;

// PWM settings
const int pwm_resolution = 14;
const int max_pwm = pow(2, pwm_resolution) - 1;

// Duty cycle conversions
const int pwm_period_us = 20000;
const int pwm_max_duty_us = 2000;
const int pwm_min_duty_us = 1000;

// Duty cycle limits
const int idle_out = max_pwm * pwm_min_duty_us / pwm_period_us;
const int min_pwm_duty = (int)(0.051 * (float)max_pwm);
const int max_pwm_duty = (int)(0.1 * (float)max_pwm);

// RC channels (0-indexed)
const int kThrottleChannel = 2; // This is Channel 3 when 1-indexed
const int kPitchChannel = 1;
const int kRollChannel = 0;

// Throttle cutoff
// Note: This is equivalent to ~1020 us input on the RC.
const float throttle_cutoff = 0.02f; // Motor power 2%

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(69, 0x28);

// Motor trim
// NOTE: this needs to be tuned after ESC calibration
// TODO: make this a config file or controlled by the RC
float fl_trim = 0.0;
float bl_trim = 0.05;
float fr_trim = 0.02;
float br_trim = 0.05;

// Command limits
const float kMaxPitch = deg_to_rad(20.0);
const float kMaxRoll = deg_to_rad(20.0);

// Filter constants
const float kGyroMix = 0.95;

// States
// Angles (rad)
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
// Angular rates (rad/s)
float roll_rate = 0.0;
float pitch_rate = 0.0;
float yaw_rate = 0.0;

// State filters
const float kAngleLpfMix = 0.2;
LowPassFilter roll_lpf(roll, kAngleLpfMix);
LowPassFilter pitch_lpf(pitch, kAngleLpfMix);
LowPassFilter yaw_lpf(yaw, kAngleLpfMix);

const float kRateLpfMix = 0.1;
LowPassFilter p_lpf(roll_rate, kRateLpfMix);
LowPassFilter q_lpf(pitch_rate, kRateLpfMix);
LowPassFilter r_lpf(yaw_rate, kRateLpfMix);

// Calibration
bool is_calibrated = false;
// Offsets (rad)
float roll_offset = 0.0;
float pitch_offset = 0.0;

// Control setpoints
float roll_setpoint = 0.0;
float pitch_setpoint = 0.0;
float yaw_setpoint = 0.0;
float p_setpoint = 0.0;
float q_setpoint = 0.0;
float r_setpoint = 0.0;

// Numerical derivative for rates
unsigned long t_prev = millis();
float roll_prev = 0.0;
float pitch_prev = 0.0;

// PID Gains
// Roll
float roll_kp = 0.05;
float roll_ki = 0.000;
float roll_kd = 0.08;
// Pitch
float pitch_kp = 0.05;
float pitch_ki = 0.00;
float pitch_kd = 0.05;
// Yaw
float yaw_kp = 0.02;
float yaw_ki = 0.0;
float yaw_kd = 0.04;

// Roll Rate
float roll_rate_kp = 0.0001;
float roll_rate_ki = 0.0000;
float roll_rate_kd = 0.004;
// Pitch Rate
float pitch_rate_kp = 0.0001;
float pitch_rate_ki = 0.00000;
float pitch_rate_kd = 0.02;
// Yaw Rate
float yaw_rate_kp = 0.005;
float yaw_rate_ki = 0.0000;
float yaw_rate_kd = 0.05;

// PID output limits
// Inner loop (rate) PID limits
const float kRatePIDLower = -0.3;
const float kRatePIDUpper = 0.3;
// Outer loop (angle) PID limits
const float kAnglePIDLower = deg_to_rad(-2.0); // rad/s
const float kAnglePIDUpper = deg_to_rad(2.0);  // rad/s

// PID Controllers
PIDController roll_pid = PIDController(roll_kp, roll_ki, roll_kd);
PIDController pitch_pid = PIDController(pitch_kp, pitch_ki, pitch_kd);
PIDController yaw_pid = PIDController(yaw_kp, yaw_ki, yaw_kd);
PIDController roll_rate_pid =
    PIDController(roll_rate_kp, roll_rate_ki, roll_rate_kd);
PIDController pitch_rate_pid =
    PIDController(pitch_rate_kp, pitch_rate_ki, pitch_rate_kd);
PIDController yaw_rate_pid =
    PIDController(yaw_rate_kp, yaw_rate_ki, yaw_rate_kd);

// Motor power
float front_left_out = 0.0;
float back_left_out = 0.0;
float front_right_out = 0.0;
float back_right_out = 0.0;

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
    float power =
        ((float)throttle - (float)pwm_min_duty_us) / (float)pwm_duty_range_us;

    return power;
}

// Convert RC throttle to percent power.
float convert_rc_stick_to_range(int &stick, const float &min, const float &max)
{
    // Get the period range
    int pwm_duty_range_us = pwm_max_duty_us - pwm_min_duty_us;

    // Convert the throttle into percentage of the maximum control duty cycle
    // This will return a value between 0 and 1 for the iFly RC.
    float range_pct =
        ((float)stick - (float)pwm_min_duty_us) / (float)pwm_duty_range_us;

    // Convert the range pct into the range
    float range_width = max - min;
    float val = min + (range_width * range_pct);

    return val;
}

// Convert the percent power for a motor to PWM duty cycle.
int convert_power_to_pwm(float power)
{
    // Get the period range
    int pwm_duty_range_us = pwm_max_duty_us - pwm_min_duty_us;

    // Power is a percentage of the maximum duty cycle period divided by the PWM
    // period
    float pwm_high_us =
        (power * (float)pwm_duty_range_us) + (float)pwm_min_duty_us;
    float duty_cycle_pct = pwm_high_us / (float)pwm_period_us;

    // The percentage is multiplied by the PWM resolution to get the analogWrite
    // value.
    float duty_cycle = duty_cycle_pct * (float)max_pwm;

    // Convert duty cycle to int for PWM output
    int pwm_duty = (int)duty_cycle;

    return pwm_duty;
}

int clamp_pwm_duty(int duty, int min_duty, int max_duty)
{
    duty = min(max_duty, max(min_duty, duty));

    return duty;
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
    if (setpoint_0_360 > yaw)
    {
        // Direct path
        delta_1 = -1.0 * (setpoint_0_360 - yaw);
        // Going around the wrap-around point
        delta_2 = (360 - setpoint_0_360) + yaw;

        // flip the direction
        if (abs(delta_2) < abs(delta_1))
        {
            delta_1 = delta_2;
        }
    }
    else
    {
        // Direct path
        delta_1 = yaw - setpoint_0_360;
        // Going around the wrap-around point
        delta_2 = (360 - yaw) + setpoint_0_360;

        // flip the direction if the second way is shorter
        if (abs(delta_2) < abs(delta_1))
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
        char name[20] = {'\0'};
        Serial.print(name);
        Serial.print("\t");
        strcat(name, log_base_name);
        strcat(name, buf);
        Serial.print("Checking: ");
        Serial.println(name);

        // If the name isn't taken, then set it.
        // Also, only allow logging if a file name exists
        if (!SD.exists(name))
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
    Serial.begin(115200);

    if (!bno.begin())
    {
        // There was a problem detecting the BNO055.
        Serial.print("No BNO055 detected - aborting flight control");
        while (1)
            ;
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
            const char *logfile = const_cast<const char *>(log_file_name);

            // Set logging headers
            File log_file = SD.open(logfile, FILE_WRITE);
            log_file.println(log_headers);
            log_file.close();
        }
        Serial.print("Logging enabled! Log file: ");
        Serial.println(log_file_name);
    }
    else
    {
        Serial.println("SD Card failed to load! Logging is not enabled.");
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
    roll_pid.SetOutputLimits(kAnglePIDLower, kAnglePIDUpper);
    pitch_pid.SetOutputLimits(kAnglePIDLower, kAnglePIDUpper);
    yaw_pid.SetOutputLimits(kAnglePIDLower, kAnglePIDUpper);
    roll_rate_pid.SetOutputLimits(kRatePIDLower, kRatePIDUpper);
    pitch_rate_pid.SetOutputLimits(kRatePIDLower, kRatePIDUpper);
    yaw_rate_pid.SetOutputLimits(kRatePIDLower, kRatePIDUpper);

    // Init the prev time to the start time.
    t_prev = micros();

    // Wait for BNO055 to initialize
    delay(5000);
}

void loop()
{

    // Run the IBusBM loop here since the interrupt
    // timing is not supported on Teensy 3.6.
    ibus.loop();

    // Read remote control channels
    for (int i = 0; i < 6; i++)
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
    if (is_rc_debug)
    {
        Serial.println();
    }

    // Get a sensor event from the BNO055 for orientation.
    sensors_event_t rpy_event;
    bno.getEvent(&rpy_event, Adafruit_BNO055::VECTOR_EULER);

    // Get a sensor event from the BNO055 for angular velocity.
    sensors_event_t pqr_event;
    bno.getEvent(&pqr_event, Adafruit_BNO055::VECTOR_GYROSCOPE);

    // Set the roll, pitch and yaw. The offsets start at 0, but then are
    // calibrated once on startup. Note: this depends on the orientation of the
    // BNO055 in the quadcopter itself. Modify as needed to achieve the correct
    // orientation.
    float roll_input = deg_to_rad(rpy_event.orientation.z) - roll_offset;
    float pitch_input =
        deg_to_rad(-1.0 * rpy_event.orientation.y) - pitch_offset;

    // Lowpass filter the angles
    roll = roll_lpf.Filter(roll_input);
    pitch = pitch_lpf.Filter(pitch_input);

    // Compute yaw as the closest angle to the yaw setpoint
    // This also converts yaw to radians
    float yaw_input = rpy_event.orientation.x;
    yaw = yaw_lpf.Filter(yaw_input);
    yaw = augment_yaw(yaw_setpoint, yaw);

    // Compute the roll and pitch rate numerically
    float dt = (float)(millis() - t_prev) / 1000.0;
    roll_rate = (float)(roll - roll_prev) / dt;
    pitch_rate = (float)(pitch - pitch_prev) / dt;

    // Set current values as previous
    t_prev = millis();
    roll_prev = roll;
    pitch_prev = pitch;

    // Set the angular rates in rad/s. Complementary filter between the
    // numerical derivative and the gyro values.
    float roll_rate_input =
        (1.0 - kGyroMix) * roll_rate + kGyroMix * deg_to_rad(pqr_event.gyro.z);
    float pitch_rate_input = (1.0 - kGyroMix) * pitch_rate +
                             kGyroMix * deg_to_rad(-1.0 * pqr_event.gyro.y);
    float yaw_rate_input =
        (1.0 - kGyroMix) * yaw_rate + kGyroMix * deg_to_rad(pqr_event.gyro.x);

    // Lowpass filter angular rates
    roll_rate = p_lpf.Filter(roll_rate_input);
    pitch_rate = q_lpf.Filter(pitch_rate_input);
    yaw_rate = r_lpf.Filter(yaw_rate_input);

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
    float throttle =
        convert_rc_throttle_to_power(channel_values[kThrottleChannel]);
    pitch_setpoint = convert_rc_stick_to_range(channel_values[kPitchChannel],
                                               -kMaxPitch, kMaxPitch);
    roll_setpoint = convert_rc_stick_to_range(channel_values[kRollChannel],
                                              -kMaxRoll, kMaxRoll);

    // The quadcopter is crashed if it rolls or pitches too much.
    if (fabs(roll) >= roll_limit || fabs(pitch) >= pitch_limit)
    {
        is_crashed = true;
    }

    // The quadcopter can only be un-crashed once the throttle is set low and
    // the quadcopter is upright
    if (throttle < throttle_cutoff && fabs(roll) < roll_limit &&
        fabs(pitch) < pitch_limit)
    {
        is_crashed = false;
    }

    // Outer loop PID controllers
    p_setpoint = roll_pid.PIDRate(roll_setpoint, roll, roll_rate);
    q_setpoint = pitch_pid.PIDRate(pitch_setpoint, pitch, pitch_rate);

    // PID control calculations
    float roll_output = -1.0 * roll_rate_pid.PID(p_setpoint, roll_rate);
    float pitch_output = pitch_rate_pid.PID(q_setpoint, pitch_rate);
    float yaw_output = yaw_rate_pid.PID(r_setpoint, yaw_rate);
    // Note: yaw_output is from the yaw rate PID, which tries to maintain 0 yaw
    // rate. Eventually an outer loop yaw controller will be added to command
    // yaw rate based on yaw setpoint.

    // If the quadcopter hasn't been calibrated, then set the roll and pitch
    // offsets. This runs at the start before flying. Only do this if the
    // quadcopter is reasonably level (within +/- 5 degrees). Otherwise the
    // calibration should not occur (fly with 0 roll/pitch offset).
    if (!is_calibrated && fabs(roll) < deg_to_rad(5.0) &&
        fabs(pitch) < deg_to_rad(5.0))
    {
        // These offsets are in radians.
        roll_offset = roll;
        pitch_offset = pitch;

        // Calibration is set
        is_calibrated = true;
    }
    else if (!is_calibrated)
    {
        // Choose to keep the 0 degree offset in order to take off from uneven
        // surfaces.
        is_calibrated = true;
    }
    else if (throttle >= throttle_cutoff && !is_crashed && is_calibrated)
    {
        // Don't do anything unless the throttle is outside a deadband, and the
        // quadcopter is uncrashed. The IMU positions must also be calibrated.

        // Compute power for each motor.
        front_left_out = throttle - roll_output + pitch_output + yaw_output;
        back_left_out = throttle - roll_output - pitch_output - yaw_output;
        front_right_out = throttle + roll_output + pitch_output - yaw_output;
        back_right_out = throttle + roll_output - pitch_output + yaw_output;

        // Apply trim and convert power to PWM
        pwm_fl = convert_power_to_pwm(front_left_out + fl_trim);
        pwm_bl = convert_power_to_pwm(back_left_out + bl_trim);
        pwm_fr = convert_power_to_pwm(front_right_out + fr_trim);
        pwm_br = convert_power_to_pwm(back_right_out + br_trim);

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
        roll_pid.Reset();
        pitch_pid.Reset();
        yaw_pid.Reset();
    }

    if (is_pwm_debug)
    {
        Serial.print("FL: ");
        Serial.print(pwm_fl);
        Serial.print(" BL: ");
        Serial.print(pwm_bl);
        Serial.print(" FR: ");
        Serial.print(pwm_fr);
        Serial.print(" BR: ");
        Serial.println(pwm_br);
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
        const char *logfile = const_cast<const char *>(log_file_name);

        // Create a data string
        String data = String(millis()) + "," + String(is_crashed) + "," +
                      String(roll, kLogPrecision) + "," +
                      String(pitch, kLogPrecision) + "," +
                      String(yaw, kLogPrecision);
        data += "," + String(roll_rate) + "," + String(pitch_rate) + "," +
                String(yaw_rate, kLogPrecision);
        data += "," + String(throttle, kLogPrecision) + "," +
                String(p_setpoint, kLogPrecision) + "," +
                String(q_setpoint, kLogPrecision) + "," +
                String(r_setpoint, kLogPrecision) + "," +
                String(roll_setpoint, kLogPrecision) + "," +
                String(pitch_setpoint, kLogPrecision) + "," +
                String(yaw_setpoint, kLogPrecision);
        data += "," + String(roll_output, kLogPrecision) + "," +
                String(pitch_output, kLogPrecision) + "," +
                String(yaw_output, kLogPrecision);
        data += "," + String(front_left_out, kLogPrecision) + "," +
                String(back_left_out, kLogPrecision) + "," +
                String(front_right_out, kLogPrecision) + "," +
                String(back_right_out, kLogPrecision);
        data += "," + String(pwm_fl) + "," + String(pwm_bl) + "," +
                String(pwm_fr) + "," + String(pwm_br);

        // Open the file, write a new line and then close.
        File log_file = SD.open(logfile, FILE_WRITE);
        log_file.println(data);
        log_file.close();
    }

    // Target 500 Hz, expect 100 Hz
    delay(2);
}
