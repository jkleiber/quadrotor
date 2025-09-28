#include "control_sim/sim_clock.h"

void SimClock::InitClock(double dt)
{
    this->t_ = 0.0;
    this->dt_ = dt;

    // Don't allow dt to be too small
    if (dt_ < 1e-10) {
        dt_ = 1e-10;
    }
}

void SimClock::Update(double t)
{
    this->t_ = t;
}

void SimClock::Increment()
{
    this->t_ += dt_;
}

void SimClock::Increment(double dt)
{
    this->t_ += dt;
}

double SimClock::GetTime() const
{
    return this->t_;
}

double SimClock::GetDt() const
{
    return this->dt_;
}