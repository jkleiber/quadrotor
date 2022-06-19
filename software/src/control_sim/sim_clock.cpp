#include "control_sim/sim_clock.h"

SimClock::SimClock()
{
    this->t_ = 0.0;
}

void SimClock::update(double t)
{
    this->t_ = t;
}

void SimClock::increment(double dt)
{
    this->t_ += dt;
}

double SimClock::get_time() const
{
    return this->t_;
}