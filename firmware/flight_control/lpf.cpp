#include "lpf.h"

float LowPassFilter::Filter(float input)
{
    state_ = alpha_ * input + (1 - alpha_) * state_;
    return state_;
}
