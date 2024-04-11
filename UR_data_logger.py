# import packages
import matplotlib.animation as animation

from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
import time
import sys
import os
import pandas as pd
from matplotlib import pyplot as plt
import datetime
from tool_force_torque import get_tcp_force_tool

# Constants
IP_ADDRESS = "172.28.60.3"
OUTPUT_FILENAME = "robot_data.csv"
FREQUENCY = 500.0

def plot_log(filename):
    df = pd.read_csv(filename)

    # Plot X
    plt.figure()
    plt.plot(df.timestamp - df.timestamp[0], df.actual_TCP_force_0)
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Force over time X')
    plt.savefig('force_x_plot.png')  # Save the plot to a file
    plt.close()

    # Plot Y
    plt.figure()
    plt.plot(df.timestamp - df.timestamp[0], df.actual_TCP_force_1)
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Force over time Y')
    plt.savefig('force_y_plot.png')  # Save the plot to a file
    plt.close()

    # Plot Z
    plt.figure()
    plt.plot(df.timestamp - df.timestamp[0], df.actual_TCP_force_2)
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Force over time Z')
    plt.savefig('force_z_plot.png')  # Save the plot to a file
    plt.close()

def plot_ft(filename):
    df = pd.read_csv(filename)

    # Plot Fx, Fy, Fz over time
    plt.figure()
    plt.plot(df['Time(s)'], df['Fx(N)'], label='Fx')
    plt.plot(df['Time(s)'], df['Fy(N)'], label='Fy')
    plt.plot(df['Time(s)'], df['Fz(N)'], label='Fz')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Force over time')
    plt.legend()
    plt.show()

    # Plot TRx, TRy, TRz over time
    plt.figure()
    plt.plot(df['Time(s)'], df['TRx(Nm)'], label='TRx')
    plt.plot(df['Time(s)'], df['TRy(Nm)'], label='TRy')
    plt.plot(df['Time(s)'], df['TRz(Nm)'], label='TRz')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('Torque over time')
    plt.legend()
    plt.show()

    # Plot bFx, bFy, bFz over time
    plt.figure()
    plt.plot(df['Time(s)'], df['bFx(N)'], label='bFx')
    plt.plot(df['Time(s)'], df['bFy(N)'], label='bFy')
    plt.plot(df['Time(s)'], df['bFz(N)'], label='bFz')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Base Force over time')
    plt.legend()
    plt.show()

    # Plot bTRx, bTRy, bTRz over time
    plt.figure()
    plt.plot(df['Time(s)'], df['bTRx(Nm)'], label='bTRx')
    plt.plot(df['Time(s)'], df['bTRy(Nm)'], label='bTRy')
    plt.plot(df['Time(s)'], df['bTRz(Nm)'], label='bTRz')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('Base Torque over time')
    plt.legend()
    plt.show()

def plot_ft_live(filename):
    def update_plot(filename):
        df = pd.read_csv(filename)
        ax.clear()  # Clear the previous plot
        ax.plot(df['Time(s)'], df['bFx(N)'], label='Fx')
        ax.plot(df['Time(s)'], df['bFy(N)'], label='Fy')
        ax.plot(df['Time(s)'], df['bFz(N)'], label='Fz')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Force [N]')
        ax.set_title('Force over time')
        ax.legend()

    # Create a Matplotlib figure and axis if they don't exist
    if 'fig' not in globals() or 'ax' not in globals():
        global fig, ax
        fig, ax = plt.subplots()
        plt.ion()  # Enable interactive mode
    update_plot(filename)
    plt.pause(0.01)  # Pause for a short time to allow plot to update

def main():
    """Main entry point."""
    dt = 1 / FREQUENCY

    rtde_r = RTDEReceive(IP_ADDRESS, FREQUENCY)
    rtde_c = RTDEControl(IP_ADDRESS, FREQUENCY)

    filename_log = f"log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    path_log = os.path.join(os.getcwd(), 'outputs', filename_log)

    filename_ft = f"ft_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    path_ft = os.path.join(os.getcwd(), 'outputs', filename_ft)

    rtde_r.startFileRecording(path_log)
    print("Data recording started, enter 'x' to end recording.")
    i = 0

    try:
        with open(path_ft, 'a') as f_force:
            f_force.write("Time(s),Fx(N),Fy(N),Fz(N),TRx(Nm),TRy(Nm),TRz(Nm),"
                          "bFx(N),bFy(N),bFz(N),bTRx(Nm),bTRy(Nm),bTRz(Nm)\n")

        while True:
            start = time.time()

            if i % 10 == 0:
                sys.stdout.write("\r")
                sys.stdout.write("{:3d} samples.".format(i))
                sys.stdout.flush()

            act_force_torque_t = rtde_r.getActualTCPForce()
            force_torque_t = get_tcp_force_tool(rtde_r, rtde_c)

            row = [i * dt * 100] + act_force_torque_t + force_torque_t

            # Write force_torque_t and act_force_torque_t to separate CSV file
            with open(path_ft, 'a') as f_force:
                f_force.write(','.join(map(str, row)) + '\n')

            end = time.time()
            duration = end - start

            if duration < dt:
                time.sleep(dt - duration)
            i += 1

            plot_ft_live(path_ft)

    except KeyboardInterrupt:
        rtde_r.stopFileRecording()
        print("\nData recording stopped.")
        # plot_log(filename_log)
        # plot_ft(filename_ft)

    finally:
        # Ensure that recording is stopped even if an exception occurs
        rtde_r.stopFileRecording()
        # plot_ft(path_ft)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Stopping recording.")
