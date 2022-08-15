#ifndef LOGGING_H_
#define LOGGING_H_

#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "control_sim/sim_clock.h"

class Logging
{
public:
    // Constructor
    Logging(SimClock *clk) : clk_(clk) {}

    // Destructor
    ~Logging();

    // Initialize logging
    void Init(std::string filename);

    void LogHeaders(std::string headers);
    void LogCsv(std::string data);
    void LogVectorXd(Eigen::VectorXd x);

    // Tell if logging is active.
    void LoggingActive(bool *active);

    // Close logging manually if desired
    void CloseLog();

private:
    // Clock
    SimClock const *clk_;

    // Log file
    std::ofstream file_;

    // If logging is active.
    bool active_;
};

#endif