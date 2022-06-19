#include "control_sim/logging.h"

void Logging::init(std::string filename)
{
    file_.open(filename);
}

void Logging::log_headers(std::string headers)
{
    // Remove whitespace from headers
    headers.erase(std::remove_if(headers.begin(), headers.end(), isspace), headers.end());
    std::cout << headers << std::endl;

    // Print header row to file
    std::string hdr_row = "time," + headers + "\n";
    file_ << hdr_row << std::flush;
}

void Logging::log_csv(std::string data)
{
    // Get time
    double t = clk_->get_time();

    // Convert time to string
    std::string t_str = std::to_string(t);

    // Write to CSV
    std::string csv_row = t_str + "," + data + "\n";
    file_ << csv_row << std::flush;
}

void Logging::log_vector_xd(Eigen::VectorXd x)
{
    int v_size = x.size();

    // Convert the vector to a comma separated string.
    std::string vector_str = "";
    for(int i = 0; i < v_size; ++i)
    {
        vector_str += std::to_string(x(i));
        if (i < (v_size - 1))
        {
            vector_str += ",";
        }
    }

    // Log this to the CSV
    this->log_csv(vector_str);
}

Logging::~Logging()
{
    file_.close();
}