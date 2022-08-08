#ifndef LOGGING_H_
#define LOGGING_H_

#include <fstream>
#include <iostream>
#include <string>

#include <eigen3/Eigen/Dense>

#include "control_sim/sim_clock.h"

class Logging
{
public:
    // Constructor
    Logging(SimClock *clk) : clk_(clk) {}

    // Destructor
    ~Logging();

    // Initialize logging
    void init(std::string filename);

    void log_headers(std::string headers);
    void log_csv(std::string data);
    void log_vector_xd(Eigen::VectorXd x);

    // Tell if logging is active.
    void logging_active(bool *active);

    // Close logging manually if desired
    void close_log();

private:
    // Clock
    SimClock const *clk_;

    // Log file
    std::ofstream file_;

    // If logging is active.
    bool active_;
};

#endif