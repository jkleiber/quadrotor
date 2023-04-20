
import numpy as np
import matplotlib.pyplot as plt


import pandas as pd

from lmfit import Parameters, Parameter, minimize

from linear_systems import simulate_system, linear_2dof_integrator


def compute_mse(ref_x, x):
    # Compute the element-wise squared error
    squared_err = np.square(ref_x - x)

    # Take the mean
    mse = np.mean(squared_err)

    return mse


def slice_data(tlm_df, t_min, t_max):
    t_slice = (tlm_df['t'] <= t_max) & (tlm_df['t'] >= t_min)

    return tlm_df[t_slice]


def cost_2dof_integrator(params, tlm_data):
    # Break data into its component parts
    t = tlm_data[:, 0]
    ctrl = tlm_data[:, 3]
    ref_traj = tlm_data[:, 1:3]

    # Find initial condition and simulate
    x0 = list(ref_traj[0, :])
    x_traj = simulate_system(t, ctrl, x0, linear_2dof_integrator, params)

    # Trajectory components
    x_rate = x_traj[1, :]

    # Reference trajectory
    ref_x_rate = ref_traj[:, 1]

    # Compute cost at each timestep (difference between true x rate and simulated x rate)
    cost = ref_x_rate - x_rate

    return cost


def sysid_quadrotor_angle(tlm_data, obj_fn):
    # Do the system ID (find the best parameters for the system)
    params = Parameters()
    params.add('a', value=-1.0, min=-1000, max=0.0)
    params.add('b', value=-20, min=-1000, max=1000.0)
    params.add('c', value=100000.0, min=0.0, max=800000.0)

    results = minimize(obj_fn, params, method='nelder',
                       args=(tlm_data,))

    # Print the parameter results
    param_star = results.params
    param_star.pretty_print()

    return param_star


def sim_quadrotor_attitude(tlm_data, sys, params, pause=False):
    t = tlm_data[:, 0]
    x = tlm_data[:, 1]
    x_rate = tlm_data[:, 2]
    actuator = tlm_data[:, 3]
    ref_traj = tlm_data[:, 1:3]

    # Simulate the identified system on the given data
    x0 = list(ref_traj[0, :])
    sys_traj = simulate_system(t, actuator, x0, sys, params)

    # Report MSE
    print(f"Angle MSE (deg): {compute_mse(x, sys_traj[0,:])}")
    print(f"Rate MSE (deg): {compute_mse(x_rate, sys_traj[1,:])}")

    # Plot the results for the time slice
    # x
    plt.figure()
    plt.plot(t, x)
    plt.plot(t, sys_traj[0, :])

    plt.title("x")
    plt.xlabel("Time (sec)")
    plt.ylabel("x (deg)")
    plt.legend(("Real", "SysID"))

    # x Rate
    plt.figure()
    plt.plot(t, x_rate)
    plt.plot(t, sys_traj[1, :])

    plt.title("x rate")
    plt.xlabel("Time (sec)")
    plt.ylabel("x rate (deg/s)")
    plt.legend(("Real", "SysID"))

    # Show plot
    plt.show(block=False)

    if pause is True:
        plt.pause(0.001)  # Pause
        input("hit[enter] to end.")
        plt.close('all')
