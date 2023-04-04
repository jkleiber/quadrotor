#include "pwm_utils.h"

float convert_rc_throttle_to_power(int throttle)
{
    // Get the period range
    int pwm_duty_range_us = kPwmMaxDutyMicros - kPwmMinDutyMicros;

    // Convert the throttle into percentage of the maximum control duty cycle
    float power =
        ((float)throttle - (float)kPwmMinDutyMicros) / (float)pwm_duty_range_us;

    return power;
}

float convert_rc_stick_to_range(int &stick, const float &min, const float &max)
{
    // Get the period range
    int pwm_duty_range_us = kPwmMaxDutyMicros - kPwmMinDutyMicros;

    // Convert the throttle into percentage of the maximum control duty cycle
    // This will return a value between 0 and 1 for the iFly RC.
    float range_pct =
        ((float)stick - (float)kPwmMinDutyMicros) / (float)pwm_duty_range_us;

    // Convert the range pct into the range
    float range_width = max - min;
    float val = min + (range_width * range_pct);

    return val;
}

// Convert the percent power for a motor to PWM duty cycle.
int convert_power_to_pwm(float power)
{
    // Get the period range
    int pwm_duty_range_us = kPwmMaxDutyMicros - kPwmMinDutyMicros;

    // Power is a percentage of the maximum duty cycle period divided by the PWM
    // period
    float pwm_high_us =
        (power * (float)pwm_duty_range_us) + (float)kPwmMinDutyMicros;
    float duty_cycle_pct = pwm_high_us / (float)kPwmPeriodMicros;

    // The percentage is multiplied by the PWM resolution to get the analogWrite
    // value.
    float duty_cycle = duty_cycle_pct * (float)kMaxPwm;

    // Convert duty cycle to int for PWM output
    int pwm_duty = (int)duty_cycle;

    return pwm_duty;
}

int clamp_pwm_duty(int duty, int min_duty, int max_duty)
{
    duty = min(max_duty, max(min_duty, duty));

    return duty;
}
