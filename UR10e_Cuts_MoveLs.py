# import packages
import rtde_control
import rtde_receive
import numpy as np
import time
import os
import datetime
import queue
import pandas as pd
from matplotlib import pyplot as plt
import sys
import threading

# UR10e 172.28.60.3
# URSIM 192.168.56.101

IP_ADDRESS = "172.28.60.3"
SPEED = 0.1
CUT_SPEED = 0.05
ACCELERATION = 0.6
FREQUENCY = 500.0

# Global variable to store force/torque offsets
force_torque_offset = [0, 0, 0, 0, 0, 0]


class UrHandler:

    def __init__(self):
        self.receive_handler = rtde_receive.RTDEReceiveInterface(IP_ADDRESS)
        self.control_handler = rtde_control.RTDEControlInterface(IP_ADDRESS)
        self.orientations = [[np.pi, 0, 0],
                             [2.261, -2.182, 0],
                             [0, np.pi, 0],
                             [2.182, 2.261, 0]]
        self.is_logging = False
        self.dt = 1 / FREQUENCY
        self.filename_log = f"log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.path_log = os.path.join(os.getcwd(), 'outputs', self.filename_log)
        self.filename_ft = f"ft_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.path_ft = os.path.join(os.getcwd(), 'outputs', self.filename_ft)
        os.makedirs('outputs', exist_ok=True)
        self.log_queue = queue.Queue()

    def get_position(self):
        x_m, y_m, z_m, rx_rad, ry_rad, rz_rad = self.receive_handler.getActualTCPPose()
        return x_m, y_m, z_m, rx_rad, ry_rad, rz_rad

    def move_to_position(self, x_m, y_m, z_m, rx_rad, ry_rad, rz_rad, speed, acceleration):
        self.control_handler.moveL([x_m, y_m, z_m, rx_rad, ry_rad, rz_rad], speed, acceleration)

    def calibrate_wrist(self):
        global force_torque_offset
        force_torque_offset = self.receive_handler.getActualTCPForce()
        print(f"Calibration complete. Force/Torque offsets: {force_torque_offset}")

    def get_calibrated_tcp_force(self):
        global force_torque_offset
        raw_force = self.receive_handler.getActualTCPForce()
        calibrated_force = [raw_force[i] - force_torque_offset[i] for i in range(6)]
        return raw_force, calibrated_force


class Position:

    def __init__(self, x, y, z, rx, ry, rz):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz


def plot_log(filename):
    df = pd.read_csv(filename)

    # Plot X
    plt.figure()
    plt.plot(df['timestamp'] - df['timestamp'][0], df['actual_TCP_force_0'])
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Force over time X')
    plt.savefig('force_x_plot.png')  # Save the plot to a file
    plt.close()

    # Plot Y
    plt.figure()
    plt.plot(df['timestamp'] - df['timestamp'][0], df['actual_TCP_force_1'])
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Force over time Y')
    plt.savefig('force_y_plot.png')  # Save the plot to a file
    plt.close()

    # Plot Z
    plt.figure()
    plt.plot(df['timestamp'] - df['timestamp'][0], df['actual_TCP_force_2'])
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Force over time Z')
    plt.savefig('force_z_plot.png')  # Save the plot to a file
    plt.close()


def plot_ft_live(path_ft):
    def update_plot():
        df = pd.read_csv(path_ft)
        ax.clear()  # Clear the previous plot
        ax.plot(df['timestamp'], df['Fx(N)'], label='Raw Fx')
        ax.plot(df['timestamp'], df['Fy(N)'], label='Raw Fy')
        ax.plot(df['timestamp'], df['Fz(N)'], label='Raw Fz')
        ax.plot(df['timestamp'], df['calibrated_Fx(N)'], label='Calibrated Fx')
        ax.plot(df['timestamp'], df['calibrated_Fy(N)'], label='Calibrated Fy')
        ax.plot(df['timestamp'], df['calibrated_Fz(N)'], label='Calibrated Fz')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Force [N]')
        ax.set_title('Force over time')
        ax.legend()
        fig.canvas.draw()  # Update the plot

    if 'fig' not in globals() or 'ax' not in globals():
        global fig, ax
        fig, ax = plt.subplots()
        plt.ion()  # Enable interactive mode

    update_plot()


def log_data(ur_handle, path_ft, stop_event):
    dt = 1 / FREQUENCY
    i = 0
    try:
        with open(path_ft, 'a') as f_force:
            f_force.write("timestamp,Fx(N),Fy(N),Fz(N),TRx(Nm),TRy(Nm),TRz(Nm),calibrated_Fx(N),calibrated_Fy(N),"
                          "calibrated_Fz(N),calibrated_TRx(Nm),calibrated_TRy(Nm),calibrated_TRz(Nm),x_m,y_m,z_m,"
                          "rx_rad,ry_rad,rz_rad,\n")

        start_time = time.time()

        while not stop_event.is_set():
            current_time = time.time() - start_time

            if i % 10 == 0:
                sys.stdout.write("\r")
                sys.stdout.write("{:3d} samples.".format(i))
                sys.stdout.flush()

            raw_force, calibrated_force = ur_handle.get_calibrated_tcp_force()
            x_m, y_m, z_m, rx_rad, ry_rad, rz_rad = ur_handle.get_position()
            pos = [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]

            row = [current_time] + raw_force + calibrated_force + pos

            # Write raw_force and calibrated_force to separate CSV file
            with open(path_ft, 'a') as f_force:
                f_force.write(','.join(map(str, row)) + '\n')

            time.sleep(dt)
            i += 1

    except KeyboardInterrupt:
        print("\nData recording stopped.")


def main() -> None:
    ur_handle = UrHandler()

    # Default behavior: execute the main sequence

    # Home position
    p1 = Position(0.5360137086330607, -0.0565907899170151, 0.06821776392308516, 1.1796574595420743,
                  1.2170222628171816, 1.2265932731417266)

    # Points for 3D printed corrugate mount
    # Down from home pos, start of travel, +X
    # p2 = Position(0.536025610454744, -0.056573229692387234, 0.01788209089141647, 1.1796270966542317,
    #               1.2170323311910154, 1.2266073646067506)
    # End of travel, -X
    # p3 = Position(0.5360277630950439, -0.19687942245054885, 0.0179165877919491, 1.1796144674583373,
    #               1.2169423423703298, 1.2266213197894118)

    p4 = Position(0.5360277630950439, -0.24248, 0.0179165877919491, 1.1796144674583373,
                  1.2169423423703298, 1.2266213197894118)

    # Points for 8020 extrusion mount
    # Down from home pos, start of travel, +X
    p2 = Position(0.5360277630950439, -0.056208284749560765, 0.035654373232785155, 1.1755687010503335,
                  1.2093688461286585, 1.2325477786282164)
    # End of travel, -X
    p3 = Position(0.5360347440587767, -0.3037507422083915, 0.03564476258752286, 1.179629144363737,
                  1.2169769934679706, 1.2266483569535254)

    filename_log = f"log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    path_log = os.path.join(os.getcwd(), 'outputs', filename_log)

    filename_ft = f"ft_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    path_ft = os.path.join(os.getcwd(), 'outputs', filename_ft)

    ur_handle.receive_handler.startFileRecording(path_log)
    print("Data recording started.")

    stop_event = threading.Event()

    # Start data logging thread
    logging_thread = threading.Thread(target=log_data, args=(ur_handle, path_ft, stop_event))
    logging_thread.start()

    try:
        # Move to positions with logging
        ur_handle.move_to_position(p1.x, p1.y, p1.z, p1.rx, p1.ry, p1.rz, SPEED, ACCELERATION)
        print("Position 1 Reached")
        time.sleep(0.5)

        # Calibrate wrist after moving to position
        ur_handle.calibrate_wrist()

        ur_handle.move_to_position(p2.x, p2.y, p2.z, p2.rx, p2.ry, p2.rz, SPEED, ACCELERATION)
        print("Position 2 Reached")
        time.sleep(0.5)

        ur_handle.move_to_position(p3.x, p3.y, p3.z, p3.rx, p3.ry, p3.rz, CUT_SPEED, ACCELERATION)
        print("Position 3 Reached")
        time.sleep(0.5)

        ur_handle.move_to_position(p1.x, p1.y, p1.z, p1.rx, p1.ry, p1.rz, SPEED, ACCELERATION)
        print("Position 1 Reached")

    except KeyboardInterrupt:
        print("\nMotion interrupted.")

    finally:
        # Stop logging
        stop_event.set()
        logging_thread.join()  # Ensure logging thread finishes
        ur_handle.receive_handler.stopFileRecording()
        plot_log(path_log)

        # Plot live data after logging is done
        plot_ft_live(path_ft)


if __name__ == '__main__':
    main()
