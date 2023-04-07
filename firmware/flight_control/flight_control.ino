// Copyright (C) 2023 Justin Kleiber

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <IBusBM.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include "lpf.h"
#include "pid.h"
#include "pwm_utils.h"

#define LOG_FN_LEN 20

float deg_to_rad(float deg) { return deg * PI / 180.0; }
float rad_to_deg(float rad) { return rad * 180.0 / PI; }

// IBus Interface
IBusBM ibus;

// Remote control values
int channel_values[6];

// SD card settings
// Note: this only works on Teensy 3.6, and the SD logging depends on the Teensy
// version of the SD library.
const int kSDChipSelect = BUILTIN_SDCARD;

// Logging
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
const float kRollLimit = 110.0;  // deg
const float kPitchLimit = 110.0; // deg

// Scope Pin
const int kScopePin = A14;

// Motor PWM pins
const int kFrontLeftMotor = 23;
const int kBackLeftMotor = 22;
const int kFrontRightMotor = 21;
const int kBackRightMotor = 20;

// RC channels (0-indexed)
const int kThrottleChannel = 2; // This is Channel 3 when 1-indexed
const int kPitchChannel = 1;
const int kRollChannel = 0;

// Remote control deadbands
// Throttle cutoff
// Note: This is equivalent to ~1020 us input on the RC.
const float kThrottleCutoff = 0.02f; // Motor power 2%
// Angular rate deadband
const float kRateDeadband = 1.0; // (deg/s)

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(69, 0x28);

// Motor trim
// NOTE: this needs to be tuned after ESC calibration
// TODO: make this a config file or controlled by the RC
float fl_trim = 0.0;
float bl_trim = 0.025;
float fr_trim = 0.00;
float br_trim = 0.025;

// Command limits
const float kMaxRollCmd = 20.0;  // deg
const float kMaxPitchCmd = 20.0; // deg

const float kMaxRollRateCmd = 50.0;  // deg
const float kMaxPitchRateCmd = 50.0; // deg
const float kMaxYawRateCmd = 20.0;   // deg

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
const float kMaxStableRoll = 5.0;  // deg
const float kMaxStablePitch = 5.0; // deg

// Calibration offsets
float roll_offset = 0.0;  // deg
float pitch_offset = 0.0; // deg

// Control setpoints
float roll_setpoint = 0.0;  // deg
float pitch_setpoint = 0.0; // deg
float yaw_setpoint = 0.0;   // deg
float p_setpoint = 0.0;     // deg/s
float q_setpoint = 0.0;     // deg/s
float r_setpoint = 0.0;     // deg/s

// Numerical derivative for rates
unsigned long t_prev = millis();
float roll_prev = 0.0;  // deg
float pitch_prev = 0.0; // deg

// PID Gains
// Roll
float roll_kp = 10;
float roll_ki = 0.000;
float roll_kd = 4.5;
// Pitch
float pitch_kp = 2.8;
float pitch_ki = 0.00;
float pitch_kd = 2.8;
// Yaw
float yaw_kp = 1.15;
float yaw_ki = 0.0;
float yaw_kd = 2.3;

// Roll Rate
float roll_rate_kp = 0.004;
float roll_rate_ki = 0.000001;
float roll_rate_kd = 0.0002;
float roll_rate_kf = -0.0001;
const float kRollRateIntegralLimit = 0.1 / roll_rate_ki;
// Pitch Rate
float pitch_rate_kp = 0.008;
float pitch_rate_ki = 0.000001;
float pitch_rate_kd = 0.0002;
float pitch_rate_kf = 0.0001;
const float kPitchRateIntegralLimit = 0.1 / pitch_rate_ki;
// Yaw Rate
float yaw_rate_kp = 0.01;
float yaw_rate_ki = 0.000001;
float yaw_rate_kd = 0.0002;
float yaw_rate_kf = 0.0001;
const float kYawRateIntegralLimit = 0.1 / yaw_rate_ki;

// PID output limits
// Inner loop (rate) PID limits
const float kRatePIDLower = -0.25; // %
const float kRatePIDUpper = 0.25;  // %

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
    if (SD.begin(kSDChipSelect))
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
    pinMode(kFrontLeftMotor, OUTPUT);
    pinMode(kBackLeftMotor, OUTPUT);
    pinMode(kFrontRightMotor, OUTPUT);
    pinMode(kBackRightMotor, OUTPUT);

    // Set the PWM resolution
    analogWriteResolution(kPwmResolution);

    // Set the PWM frequencies for each motor to 50 Hz.
    analogWriteFrequency(kFrontLeftMotor, 50);
    analogWriteFrequency(kBackLeftMotor, 50);
    analogWriteFrequency(kFrontRightMotor, 50);
    analogWriteFrequency(kBackRightMotor, 50);

    // Set PID output limits
    roll_pid.SetOutputLimits(-kMaxRollRateCmd, kMaxRollRateCmd);
    pitch_pid.SetOutputLimits(-kMaxPitchRateCmd, kMaxPitchRateCmd);
    yaw_pid.SetOutputLimits(-kMaxYawRateCmd, kMaxYawRateCmd);
    roll_rate_pid.SetOutputLimits(kRatePIDLower, kRatePIDUpper);
    pitch_rate_pid.SetOutputLimits(kRatePIDLower, kRatePIDUpper);
    yaw_rate_pid.SetOutputLimits(kRatePIDLower, kRatePIDUpper);

    // Set the PID integrator limits
    roll_rate_pid.SetIntegratorLimits(-kRollRateIntegralLimit,
                                      kRollRateIntegralLimit);
    pitch_rate_pid.SetIntegratorLimits(-kPitchRateIntegralLimit,
                                       kPitchRateIntegralLimit);
    yaw_rate_pid.SetIntegratorLimits(-kYawRateIntegralLimit,
                                     kYawRateIntegralLimit);

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
    float roll_input = rpy_event.orientation.z - roll_offset;
    float pitch_input = (-1.0 * rpy_event.orientation.y) - pitch_offset;

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
        (1.0 - kGyroMix) * roll_rate + kGyroMix * pqr_event.gyro.z;
    float pitch_rate_input =
        (1.0 - kGyroMix) * pitch_rate + kGyroMix * (-1.0 * pqr_event.gyro.y);
    float yaw_rate_input =
        (1.0 - kGyroMix) * yaw_rate + kGyroMix * pqr_event.gyro.x;

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
    // HACK: tune PIDs using "rate" mode
    // roll_setpoint = convert_rc_stick_to_range(channel_values[kRollChannel],
    //                                           -kMaxRollCmd, kMaxRollCmd);
    // pitch_setpoint = convert_rc_stick_to_range(channel_values[kPitchChannel],
    //                                            -kMaxPitchCmd, kMaxPitchCmd);

    // The quadcopter is crashed if it rolls or pitches too much.
    if (fabs(roll) >= kRollLimit || fabs(pitch) >= kPitchLimit)
    {
        is_crashed = true;
    }

    // The quadcopter can only be un-crashed once the throttle is set low and
    // the quadcopter is upright
    if (throttle < kThrottleCutoff && fabs(roll) < kRollLimit &&
        fabs(pitch) < kPitchLimit)
    {
        is_crashed = false;
    }

    // Outer loop PID controllers
    // p_setpoint = roll_pid.PIDRate(roll_setpoint, roll, roll_rate);
    // q_setpoint = pitch_pid.PIDRate(pitch_setpoint, pitch, pitch_rate);
    p_setpoint = convert_rc_stick_to_range(channel_values[kRollChannel],
                                           -kMaxRollRateCmd, kMaxRollRateCmd);
    q_setpoint = convert_rc_stick_to_range(channel_values[kPitchChannel],
                                           -kMaxPitchRateCmd, kMaxPitchRateCmd);

    // Apply deadbands.
    if (fabs(p_setpoint) < kRateDeadband)
    {
        p_setpoint = 0.0;
    }
    if (fabs(q_setpoint) < kRateDeadband)
    {
        q_setpoint = 0.0;
    }

    // PID control calculations
    float roll_output = -1.0 * roll_rate_pid.PID(p_setpoint, roll_rate) +
                        (p_setpoint * roll_rate_kf);
    float pitch_output = pitch_rate_pid.PID(q_setpoint, pitch_rate) +
                         (q_setpoint * pitch_rate_kf);
    float yaw_output =
        yaw_rate_pid.PID(r_setpoint, yaw_rate) + (r_setpoint * yaw_rate_kf);
    // Note: yaw_output is from the yaw rate PID, which tries to maintain 0 yaw
    // rate. Eventually an outer loop yaw controller will be added to command
    // yaw rate based on yaw setpoint.

    // If the quadcopter hasn't been calibrated, then set the roll and pitch
    // offsets. This runs at the start before flying. Only do this if the
    // quadcopter is reasonably level (within +/- 5 degrees). Otherwise the
    // calibration should not occur (fly with 0 roll/pitch offset).
    if (!is_calibrated && fabs(roll) < kMaxStableRoll &&
        fabs(pitch) < kMaxStablePitch)
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
    else if (throttle >= kThrottleCutoff && !is_crashed && is_calibrated)
    {
        // Don't do anything unless the throttle is outside a deadband, and the
        // quadcopter is uncrashed. The IMU positions must also be calibrated.

        // MOTOR MAPPING
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
        pwm_fl = clamp_pwm_duty(pwm_fl, kPwmMinDutyMicros, kPwmMaxDutyMicros);
        pwm_bl = clamp_pwm_duty(pwm_bl, kPwmMinDutyMicros, kPwmMaxDutyMicros);
        pwm_fr = clamp_pwm_duty(pwm_fr, kPwmMinDutyMicros, kPwmMaxDutyMicros);
        pwm_br = clamp_pwm_duty(pwm_br, kPwmMinDutyMicros, kPwmMaxDutyMicros);

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

        pwm_fl = kIdlePwm;
        pwm_bl = kIdlePwm;
        pwm_fr = kIdlePwm;
        pwm_br = kIdlePwm;

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
    analogWrite(kFrontLeftMotor, pwm_fl);
    analogWrite(kBackLeftMotor, pwm_bl);
    analogWrite(kFrontRightMotor, pwm_fr);
    analogWrite(kBackRightMotor, pwm_br);

    if (is_scope_debug)
    {
        int scope_input = analogRead(kScopePin);
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
