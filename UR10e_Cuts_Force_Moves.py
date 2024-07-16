# import packages
import rtde_control
import rtde_receive
import numpy as np
import gc
import argparse
import time
import sys
import os
import pandas as pd
from matplotlib import pyplot as plt
import datetime
from tool_force_torque import get_tcp_force_tool
import threading
import queue

# UR10e 172.28.60.3
# URSIM 192.168.56.101

IP_ADDRESS = "172.28.60.3"
SPEED = 0.01
ACCELERATION = .5
FREQUENCY = 1000.0

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

    def move_to_position(self, x_m, y_m, z_m, rx_rad, ry_rad, rz_rad):
        self.control_handler.moveL([x_m, y_m, z_m, rx_rad, ry_rad, rz_rad], SPEED, ACCELERATION)

    def calibrate_wrist(self):
        global force_torque_offset
        force_torque_offset = self.receive_handler.getActualTCPForce()
        print(f"Calibration complete. Force/Torque offsets: {force_torque_offset}")

    def get_calibrated_tcp_force(self):
        global force_torque_offset
        raw_force = self.receive_handler.getActualTCPForce()
        calibrated_force = [raw_force[i] - force_torque_offset[i] for i in range(6)]
        return calibrated_force

    def f_move_1d(self, direction=None, force_threshold=10, move_step=0.01, speed=0.15, acceleration=1.2):
        """
        Move in a Cartesian direction using force mode until the force measured at the tool center point (TCP) reaches the specified
        threshold.

        :param direction: Direction to move in ('x', 'y', 'z', '-x', '-y', '-z')
        :param force_threshold: Force threshold to trigger the stop in Newtons
        :param move_step: Step size in meters for each move iteration
        :param speed: Speed at which the robot should move (in meters per second)
        :param acceleration: Acceleration at which the robot should move (in meters per second squared)

        Example:
        ur_handler.f_move_1d(direction='z', force_threshold=5, move_step=0.01, speed=0.15, acceleration=1.2)
        Moves the robot in the z-direction until the force in the z-direction exceeds 5 Newtons, with each step being
        0.01 meters.
        """
        direction = direction.lower()
        assert direction in ['x', 'y', 'z', '-x', '-y',
                             '-z'], "Direction must be one of 'x', 'y', 'z', '-x', '-y', '-z'"

        direction_index = {'x': 0, 'y': 1, 'z': 2, '-x': 0, '-y': 1, '-z': 2}[direction]
        direction_sign = 1 if direction[0] != '-' else -1

        # Get current position
        current_position = list(self.get_position())

        # Set up force vector and selection vector
        force_vector = [0, 0, 0, 0, 0, 0]
        selection_vector = [0, 0, 0, 0, 0, 0]
        limits = [1000, 1000, 1000, 1000, 1000, 1000]

        force_vector[direction_index] = direction_sign * force_threshold
        selection_vector[direction_index] = 1

        self.control_handler.forceMode(current_position[:3], selection_vector, force_vector, 2, limits)

        while True:
            force = self.get_calibrated_tcp_force()
            if abs(force[direction_index]) > abs(force_threshold):
                print("Contact detected.")
                break

            # Update position
            current_position[direction_index] += direction_sign * move_step
            self.control_handler.moveL(current_position, speed, acceleration)

            # Adjust force mode
            self.control_handler.forceMode(current_position[:3], selection_vector, force_vector, 2, limits)

        # Reset force mode parameters to zero
        self.control_handler.forceMode(current_position[:3], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], 2, limits)

    def f_move_2d(self, move_direction, move_distance, force_direction, force_magnitude):
        """
        Combination move, specifying both the direction to move in and the direction to apply force in.

        :param move_direction: Direction to move in ('x', 'y', 'z')
        :param move_distance: Distance to move in meters
        :param force_direction: Direction to apply force in ('x', 'y', 'z')
        :param force_magnitude: Magnitude of force to apply in Newtons

        Example:
        ur_handler.f_move_2d(move_direction='x', move_distance=-0.1, force_direction='z', force_magnitude=10)
        Moves the robot in the -x direction by 100 mm while applying a 10 N force in the z direction.
        """
        move_direction = move_direction.lower()
        force_direction = force_direction.lower()
        assert move_direction in ['x', 'y', 'z'], "Move direction must be one of 'x', 'y', 'z'"
        assert force_direction in ['x', 'y', 'z'], "Force direction must be one of 'x', 'y', 'z'"

        move_index = {'x': 0, 'y': 1, 'z': 2}[move_direction]
        force_index = {'x': 0, 'y': 1, 'z': 2}[force_direction]

        current_position = list(self.get_position())
        target_position = current_position[:]
        target_position[move_index] += move_distance

        # Set up force vector and selection vector
        force_vector = [0, 0, 0, 0, 0, 0]
        force_vector[force_index] = force_magnitude

        selection_vector = [0, 0, 0, 0, 0, 0]
        selection_vector[force_index] = 1

        limits = [1000] * 6  # Set high limits for forces and torques

        # Apply force mode and move to target position
        self.control_handler.forceMode(current_position[:3], selection_vector, force_vector, 2, limits)
        self.control_handler.moveL(target_position, SPEED, ACCELERATION)
        self.control_handler.forceMode([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], 2,
                                       [1000, 1000, 1000, 1000, 1000, 1000])

    def start_logging(self):
        self.is_logging = True
        self.receive_handler.startFileRecording(self.path_log)
        threading.Thread(target=self.log_data).start()

    def stop_logging(self):
        self.is_logging = False
        self.receive_handler.stopFileRecording()

    def log_data(self):
        dt = self.dt
        i = 0

        with open(self.path_ft, 'a') as f_force:
            f_force.write("Time(s),Fx(N),Fy(N),Fz(N),TRx(Nm),TRy(Nm),TRz(Nm),"
                          "bFx(N),bFy(N),bFz(N),bTRx(Nm),bTRy(Nm),bTRz(Nm)\n")

        try:
            while self.is_logging:
                start = time.time()

                if i % 10 == 0:
                    sys.stdout.write("\r")
                    sys.stdout.write("{:3d} samples.".format(i))
                    sys.stdout.flush()

                act_force_torque_t = self.get_calibrated_tcp_force()
                try:
                    force_torque_t = get_tcp_force_tool(self.receive_handler, self.control_handler)
                except RuntimeError as e:
                    print(f"Error getting force torque tool data: {e}")
                    force_torque_t = [0, 0, 0, 0, 0, 0]

                row = [i * dt * 100] + act_force_torque_t + force_torque_t

                with open(self.path_ft, 'a') as f_force:
                    f_force.write(','.join(map(str, row)) + '\n')

                self.log_queue.put(row)  # Send data to the queue for plotting

                end = time.time()
                duration = end - start

                if duration < dt:
                    time.sleep(dt - duration)
                i += 1

        except KeyboardInterrupt:
            print("\nData recording stopped.")
        finally:
            # Ensure that recording is stopped even if an exception occurs
            self.receive_handler.stopFileRecording()

    def plot_ft_live(self):
        def update_plot(data):
            df = pd.DataFrame(data, columns=["Time(s)", "Fx(N)", "Fy(N)", "Fz(N)", "TRx(Nm)", "TRy(Nm)", "TRz(Nm)",
                                             "bFx(N)", "bFy(N)", "bFz(N)", "bTRx(Nm)", "bTRy(Nm)", "bTRz(Nm)"])
            ax.clear()  # Clear the previous plot
            ax.plot(df['Time(s)'], df['Fx(N)'], label='Fx')
            ax.plot(df['Time(s)'], df['Fy(N)'], label='Fy')
            ax.plot(df['Time(s)'], df['Fz(N)'], label='Fz')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Force [N]')
            ax.set_title('Force over time')
            ax.legend()

            # Set dynamic x-axis limits
            ax.set_xlim(df['Time(s)'].min(), df['Time(s)'].max())

        if 'fig' not in globals() or 'ax' not in globals():
            global fig, ax
            fig, ax = plt.subplots()
            plt.ion()  # Enable interactive mode

        start_time = time.time()

        while self.is_logging:
            try:
                data = []
                while not self.log_queue.empty():
                    row = self.log_queue.get_nowait()
                    row[0] = time.time() - start_time  # Update time to be relative to start
                    data.append(row)
                if data:
                    update_plot(data)
                    plt.pause(0.01)  # Pause for a short time to allow plot to update
            except queue.Empty:
                pass

class Position:

    def __init__(self, x, y, z, rx, ry, rz):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz

def main() -> None:
    # Initialize the plotting environment before starting threads
    global fig, ax
    fig, ax = plt.subplots()
    plt.ion()  # Enable interactive mode

    parser = argparse.ArgumentParser(description="UR10e Robot Controller")
    subparsers = parser.add_subparsers(dest="command")

    # Subparser for get_position command
    subparsers.add_parser("get_position", help="Get the current position of the robot")

    # Subparser for move_to_position command
    move_to_position_parser = subparsers.add_parser("move_to_position", help="Move to a specific position")
    move_to_position_parser.add_argument("x", type=float, help="X coordinate in meters")
    move_to_position_parser.add_argument("y", type=float, help="Y coordinate in meters")
    move_to_position_parser.add_argument("z", type=float, help="Z coordinate in meters")
    move_to_position_parser.add_argument("rx", type=float, help="Rotation around X axis in radians")
    move_to_position_parser.add_argument("ry", type=float, help="Rotation around Y axis in radians")
    move_to_position_parser.add_argument("rz", type=float, help="Rotation around Z axis in radians")

    # Subparser for calibrate_wrist command
    subparsers.add_parser("calibrate_wrist", help="Calibrate the wrist load cell")

    # Subparser for start_logging command
    subparsers.add_parser("start_logging", help="Start logging force/torque data")

    # Subparser for stop_logging command
    subparsers.add_parser("stop_logging", help="Stop logging force/torque data")

    # Parse arguments
    args = parser.parse_args()

    ur_handle = UrHandler()

    if args.command == "get_position":
        position = ur_handle.get_position()
        print(f"Current position: {position}")

    elif args.command == "move_to_position":
        ur_handle.move_to_position(args.x, args.y, args.z, args.rx, args.ry, args.rz)

    elif args.command == "calibrate_wrist":
        ur_handle.calibrate_wrist()

    elif args.command == "start_logging":
        ur_handle.start_logging()
        ur_handle.plot_ft_live()

    elif args.command == "stop_logging":
        ur_handle.stop_logging()

    else:
        # Default behavior: execute the main sequence
        p1 = Position(0.5623521063322213, -0.05660109875831779, 0.06824539528791637, 1.179620409598821,
                      1.2168988007661623, 1.2266470473548827)
        ur_handle.move_to_position(p1.x, p1.y, p1.z, p1.rx, p1.ry, p1.rz)

        # Calibrate wrist after moving to position
        ur_handle.calibrate_wrist()

        # Start logging data
        ur_handle.start_logging()

        print("Calibration complete. Starting test sequence.")
        time.sleep(2)

        # Move down until contact
        ur_handle.f_move_1d(direction='-z', force_threshold=10)
        print("Contact detected.")

        time.sleep(2)

        print("Initializing 2D move")
        # Move in -x direction for 100 mm while applying 10N in z
        ur_handle.f_move_2d(move_direction='x', move_distance=-0.1, force_direction='z', force_magnitude=10)

        # Stop logging data
        ur_handle.stop_logging()

        print("Data logging stopped. Test Complete.")


if __name__ == '__main__':
    main()
