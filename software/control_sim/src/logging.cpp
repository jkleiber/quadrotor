#include "control_sim/logging.h"

void Logging::Init(std::string filename)
{
    active_ = true;
    file_.open(filename, std::ofstream::out | std::ofstream::trunc);
}

void Logging::LogHeaders(std::string headers)
{
    // Remove whitespace from headers
    headers.erase(std::remove_if(headers.begin(), headers.end(), isspace),
                  headers.end());
    std::cout << headers << std::endl;

    // Print header row to file
    std::string hdr_row = "time," + headers + "\n";
    file_ << hdr_row << std::flush;
}

void Logging::LogCsv(std::string data)
{
    // Get time
    double t = clk_->GetTime();

    // Convert time to string
    std::string t_str = std::to_string(t);

    // Write to CSV
    std::string csv_row = t_str + "," + data + "\n";
    file_ << csv_row << std::flush;
}

void Logging::LogVectorXd(Eigen::VectorXd x)
{
    int v_size = x.size();

    // Convert the vector to a comma separated string.
    std::string vector_str = "";
    for (int i = 0; i < v_size; ++i)
    {
        vector_str += std::to_string(x(i));
        if (i < (v_size - 1))
        {
            vector_str += ",";
        }
    }

    // Log this to the CSV
    this->LogCsv(vector_str);
}

void Logging::LoggingActive(bool *active)
{
    if (active != nullptr)
    {
        *active = active_;
    }
}

void Logging::CloseLog()
{
    file_.close();
    active_ = false;
}

Logging::~Logging() { file_.close(); }