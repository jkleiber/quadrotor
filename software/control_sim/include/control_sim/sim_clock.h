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

        // Initialize the clock and the Increment
        void InitClock(double dt);

        void Update(double t);
        void Increment();
        void Increment(double dt);

        double GetTime() const;
        double GetDt() const;

    private:
        double t_;

        double dt_;
};

#endif