# import packages
import rtde_control
import rtde_receive
import numpy as np
import time
import os
import datetime
import queue

# UR10e 172.28.60.3
# URSIM 192.168.56.101

IP_ADDRESS = "172.28.60.3"
SPEED = 0.05
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

    def move_until_force_exceeded(self, direction, force_threshold):
        # Set the desired direction vector for the TCP to move in
        # direction should be a list of 6 elements [x, y, z, Rx, Ry, Rz]
        # force_threshold is the force value in Newtons

        # Get current position
        current_pose = self.receive_handler.getActualTCPPose()

        # Move a small distance in the desired direction to initiate force mode
        target_pose = [
            current_pose[0] + direction[0] * 0.1,
            current_pose[1] + direction[1] * 0.1,
            current_pose[2] + direction[2] * 0.1,
            current_pose[3] + direction[3] * 0.1,
            current_pose[4] + direction[4] * 0.1,
            current_pose[5] + direction[5] * 0.1,
        ]

        self.control_handler.moveL(target_pose, SPEED, ACCELERATION)

        # Define the compliance parameters
        compliance_frame = [0, 0, 0, 0, 0, 0]
        force = [0, 0, 0, 0, 0, 0]  # No specific force vector
        type = 2  # force mode type 2 (move in task space)
        limits = [10, 10, 10, 1, 1, 1]  # Compliance limits

        # Enter force mode
        print("Entering force mode")
        self.control_handler.forceMode(compliance_frame, direction, force, type, limits)

        while True:
            # Read the current calibrated force values
            current_force = self.get_calibrated_tcp_force()

            # Calculate the magnitude of the force
            force_magnitude = np.linalg.norm([current_force[0], current_force[1], current_force[2]])

            # Debugging output for current force and magnitude
            print(f"Current calibrated force: {current_force}")
            print(f"Calibrated force magnitude: {force_magnitude}")

            # Check if the force threshold is exceeded
            if force_magnitude > force_threshold:
                # Exit force mode and stop the movement
                print("Force threshold exceeded, stopping movement")
                self.control_handler.forceModeStop()
                self.control_handler.stopL(2)
                break

            # Small delay to prevent busy-waiting
            time.sleep(0.01)

        # Exit force mode (as a precaution in case of an error in the loop)
        self.control_handler.forceModeStop()

class Position:

    def __init__(self, x, y, z, rx, ry, rz):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz


def main() -> None:

    ur_handle = UrHandler()

    # Default behavior: execute the main sequence
    p1 = Position(0.5623521063322213, -0.05660109875831779, 0.06824539528791637, 1.179620409598821,
                  1.2168988007661623, 1.2266470473548827)
    ur_handle.move_to_position(p1.x, p1.y, p1.z, p1.rx, p1.ry, p1.rz)

    # Calibrate wrist after moving to position
    ur_handle.calibrate_wrist()

    # Move down until contact
    ur_handle.move_until_force_exceeded([0, 0, -1, 0, 0, 0], 8)


if __name__ == '__main__':
    main()
