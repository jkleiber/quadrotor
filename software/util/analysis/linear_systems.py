
import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d


def simulate_system(t, ctrl, x0, linear_sys, params):
    # Find u(t) at using interpolation
    t0 = t[0]
    t = t - t0
    u_t = interp1d(t, ctrl)

    # Find the end time for simulation
    tf = np.max(t)

    # Simulate an arbitrary linear system given system ID parameters.
    x_traj = solve_ivp(linear_sys, t_span=[
                       0, tf], y0=x0, t_eval=t, args=(u_t, params))

    return x_traj.y


def linear_2dof_integrator(t, x, u_t, params):
    # Use the interpolation function to get the control at time t
    u = u_t(t)

    # Parameters for x
    a = params['a']
    b = params['b']
    c = params['c']

    # Linear state equation
    A = np.array([[0, 1], [a, b]])
    B = np.array([[0], [c]])

    # Make x into a column vector for matrix multiplication
    X = x[None].T

    # System update
    x_dot = (A@X + B*u).T.flatten()

    return x_dot
