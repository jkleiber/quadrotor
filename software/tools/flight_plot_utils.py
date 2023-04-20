
import pandas as pd
import matplotlib.pyplot as plt


def load_csv(file):
    # File locations
    log_path = "../logs/" + file

    csv_df = pd.read_csv(log_path)
    csv_df.columns = csv_df.columns.str.replace(" ", "")

    return csv_df


def plot_rate(t_clip, cmd, x, x_rate, pid=None, label=None):
    if pid is None:
        f, axs = plt.subplots(2, 1, sharex=True)
    else:
        f, axs = plt.subplots(2, 2, sharex=True)

    state_name = "angle"
    if label is not None:
        state_name = label

    axs[0, 0].plot(t_clip, x, label=f"{state_name}")
    axs[0, 0].plot(t_clip, cmd, label=f"{state_name} cmd")
    axs[0, 0].legend()
    axs[0, 0].set_ylabel(f"{state_name} (deg/s)")

    axs[1, 0].plot(t_clip, x_rate, label=f"{state_name} rate")
    axs[1, 0].set_ylabel(f"{state_name} rate (deg/s^2)")

    if pid is not None:
        axs[0, 1].plot(t_clip, cmd - x, label=f"{state_name} error")
        axs[0, 1].set_ylabel(f"{state_name} Error (deg)")

        axs[1, 1].plot(t_clip, pid, label=f"{state_name} PID")
        axs[1, 1].set_ylabel(f"{state_name} PID (%)")

    plt.tight_layout()
    plt.show()


def plot_angle(t_clip, cmd, x, x_rate, pid=None, label=None):
    if pid is None:
        f, axs = plt.subplots(2, sharex=True)
    else:
        f, axs = plt.subplots(3, sharex=True)

    state_name = "angle"
    if label is not None:
        state_name = label

    axs[0].plot(t_clip, x, label=f"{state_name}")
    axs[0].plot(t_clip, cmd, label=f"{state_name} cmd")
    axs[0].legend()
    axs[0].set_ylabel(f"{state_name} (deg)")

    axs[1].plot(t_clip, x_rate, label=f"{state_name} rate")
    axs[1].set_ylabel(f"{state_name} rate (deg/s)")

    if pid is not None:
        axs[2].plot(t_clip, pid, label=f"{state_name} PID")
        axs[2].set_ylabel(f"{state_name} PID (deg/s)")

    plt.show()
