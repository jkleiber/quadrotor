
#pragma once

#include <Arduino.h>

// PWM settings
const int kPwmResolution = 14;
const int kMaxPwm = pow(2, kPwmResolution) - 1;

// Duty cycle conversions
const int kPwmPeriodMicros = 20000;
const int kPwmMaxDutyMicros = 2000;
const int kPwmMinDutyMicros = 1000;

// Duty cycle limits
const int kIdlePwm = kMaxPwm * kPwmMinDutyMicros / kPwmPeriodMicros;
const int kMinPwmDuty = (int)(0.051 * (float)kMaxPwm);
const int kMaxPwmDuty = (int)(0.1 * (float)kMaxPwm);

/**
 * @brief Convert the throttle stick to a power percentage [0, 100]%
 *
 * @param throttle Raw throttle stick PWM (between 1000-2000)
 * @return float Throttle power
 */
float convert_rc_throttle_to_power(int throttle);

/**
 * @brief Convert any remote control stick value to a defined range.
 *
 * @param stick Raw stick PWM (between 1000-2000)
 * @param min Minimum value for range to convert to
 * @param max Maximum value for range to convert to
 * @return float Stick value converted to a range
 */
float convert_rc_stick_to_range(int &stick, const float &min, const float &max);

/**
 * @brief Convert a 0-1 power percentage into PWM duty cycle for a brushless
 * quadcopter motor.
 *
 * @param power Power from 0-1.
 * @return int PWM duty cycle in microseconds.
 */
int convert_power_to_pwm(float power);

/**
 * @brief Clamp the duty cycle to a prescribed range.
 *
 * @param duty A given duty cycle for a motor.
 * @param min_duty Minimum duty cycle allowed.
 * @param max_duty Maximum duty cycle allowed.
 * @return int The clamped duty cycle.
 */
int clamp_pwm_duty(int duty, int min_duty, int max_duty);
