#ifndef LOGGING_H_
#define LOGGING_H_

#include <string>
#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>

#include "control_sim/sim_clock.h"

class Logging
{
    public:
        // Constructor
        Logging(std::string filename, SimClock* clk) : clk_(clk) 
        {
            this->init(filename);
        }

        // Destructor
        ~Logging();

        // Initialize logging
        void init(std::string filename);

        void log_headers(std::string headers);
        void log_csv(std::string data);
        void log_vector_xd(Eigen::VectorXd x);

    private:
        // Clock
        SimClock const* clk_;

        // Log file
        std::ofstream file_;
};

#endif