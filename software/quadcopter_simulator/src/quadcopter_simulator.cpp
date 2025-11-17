#include "quadcopter_simulator/quadcopter_simulator.h"

void QuadcopterSim::Init()
{
    // Set time to 0.
    clk_->Update(0.0);

    // Previous print at -100 to print initial conditions
    prev_print_time_ = -100.0;

    // Initial Conditions
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);

    // Initialize dynamics
    quadcopter_.Init(x0);

    // Initialize logging.
    InitLogging();
}

void QuadcopterSim::InitLogging()
{
    // Initialize logging
    if (state_log_.IsActive())
    {
        state_log_.CloseLog();
    }

    // TODO: time based logging
    state_log_.Init("state_log.csv");
    state_log_.LogHeaders("u, v, w, p, q, r, x, y, z, roll, pitch, yaw");
}

void QuadcopterSim::Update()
{
    // Print periodically while the simulation runs
    if (clk_->GetTime() - prev_print_time_ > 1.0)
    {
        Eigen::VectorXd x = quadcopter_.GetState();
        prev_print_time_ = clk_->GetTime();
        std::cout << "t=" << prev_print_time_ << "\troll=" << x(9)
                  << " pitch=" << x(10) << " yaw=" << x(11) << std::endl;
    }

    // Control loop
    Eigen::VectorXd motors = Eigen::VectorXd::Zero(4);

    // Quadcopter dynamics
    quadcopter_.UpdateDynamics(motors);

    // Log the current sim state.
    LogState();

    // Increment time
    clk_->Increment();
}

void QuadcopterSim::LogState()
{
    // Log the current state.
    state_log_.LogVectorXd(quadcopter_.GetState());
}
