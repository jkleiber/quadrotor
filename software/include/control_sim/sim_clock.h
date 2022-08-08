#ifndef SIM_CLOCK_H
#define SIM_CLOCK_H

class SimClock 
{
    public:
        SimClock() {
            InitClock(0.01);
        }
        SimClock(double dt){
            InitClock(dt);
        }

        // Initialize the clock and the increment
        void InitClock(double dt);

        void update(double t);
        void increment();
        void increment(double dt);

        double get_time() const;
        double get_dt() const;

    private:
        double t_;

        double dt_;
};

#endif