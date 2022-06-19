#ifndef SIM_CLOCK_H
#define SIM_CLOCK_H

class SimClock 
{
    public:
        SimClock();

        void update(double t);
        void increment(double dt);

        double get_time() const;

    private:
        double t_;
};

#endif