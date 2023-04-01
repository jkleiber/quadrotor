
#pragma once

class LowPassFilter
{
public:
    LowPassFilter(float x0, float alpha) : state_(x0), alpha_(alpha) {}
    ~LowPassFilter() {}

    float Filter(float input);

private:
    float alpha_;

    float state_;
};
