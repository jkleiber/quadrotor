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

void SimClock::update(double t)
{
    this->t_ = t;
}

void SimClock::increment()
{
    this->t_ += dt_;
}

void SimClock::increment(double dt)
{
    this->t_ += dt;
}

double SimClock::get_time() const
{
    return this->t_;
}

double SimClock::get_dt() const
{
    return this->dt_;
}