#ifndef ATTITUDE_CONTROL_PARAMETERS_H_
#define ATTITUDE_CONTROL_PARAMETERS_H_

enum ControlMode
{
    Rate = 0,
    Angle = 1
};
const ControlMode kMode = Rate;

// Crash detection
const float kRollLimit = 110.0;  // deg
const float kPitchLimit = 110.0; // deg

// Calibration
const float kMaxStableRoll = 5.0;  // deg
const float kMaxStablePitch = 5.0; // deg

// Scope Pin
// const int kScopePin = A14;

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
const float kThrottleCutoff = 0.1f; // Motor power 10%

// Command limits
const float kMaxRollCmd = 20.0;  // deg
const float kMaxPitchCmd = 20.0; // deg

const float kMaxRollRateCmd = 200.0;  // deg
const float kMaxPitchRateCmd = 200.0; // deg
const float kMaxYawRateCmd = 100.0;   // deg

// PID Gains
// Roll
const float kRollKp = 0.05;
const float kRollKi = 0.00001;
const float kRollKd = 0.001;
const float kRollLeak = 0.99;
// Pitch
const float kPitchKp = 0.07;
const float kPitchKi = 0.0001;
const float kPitchKd = 0.001;
const float kPitchLeak = 0.99;
// Yaw
const float kYawKp = 0.06;
const float kYawKi = 0.0001;
const float kYawKd = 0.001;
const float kYawLeak = 0.99;

// Roll Rate
const float kRollRatePDRatio = 40;
const float kRollRateKp = -0.001;
const float kRollRateKi = -0.00000001;
const float kRollRateKd = kRollRateKp / kRollRatePDRatio;
const float kRollRateKf = -0.0001;
const float kRollRateLeak = 0.99;
const float kRollRateIntegralLimit = 0.1 / kRollRateKi;
// Pitch Rate
const float kPitchRatePDRatio = 2 * 10;
const float kPitchRateKp = 0.0006;
const float kPitchRateKi = 0.00000001;
const float kPitchRateKd = kPitchRateKp / kPitchRatePDRatio;
const float kPitchRateKf = 0.0001;
const float kPitchRateLeak = 0.99;
const float kPitchRateIntegralLimit = 0.1 / kPitchRateKi;
// Yaw Rate
const float kYawRatePDRatio = 2 * 10;
const float kYawRateKp = 0.0006;
const float kYawRateKi = 0.00000001;
const float kYawRateKd = kYawRateKp / kYawRatePDRatio;
const float kYawRateKf = 0.0001;
const float kYawRateLeak = 0.99;
const float kYawRateIntegralLimit = 0.1 / kYawRateKi;

// PID output limits
// Inner loop (rate) PID limits
const float kRatePIDLower = -0.25; // %
const float kRatePIDUpper = 0.25;  // %

// Angular rate deadband
const float kRateDeadband = 10.0; // (deg/s)

// Throttle curve parameters
const float kMinThrottle = 0.22;

#endif
