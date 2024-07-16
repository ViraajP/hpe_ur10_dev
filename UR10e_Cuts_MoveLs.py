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
        return calibrated_force


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
    p1 = Position(0.5360137086330607, -0.0565907899170151, 0.06821776392308516, 1.1796574595420743,
                  1.2170222628171816, 1.2265932731417266)
    p2 = Position(0.536025610454744, -0.056573229692387234, 0.01788209089141647, 1.1796270966542317,
                  1.2170323311910154, 1.2266073646067506)
    p3 = Position(0.5360277630950439, -0.19687942245054885, 0.0179165877919491, 1.1796144674583373,
                  1.2169423423703298, 1.2266213197894118)

    ur_handle.move_to_position(p1.x, p1.y, p1.z, p1.rx, p1.ry, p1.rz, SPEED, ACCELERATION)
    print("Position 1 Reached")
    time.sleep(1)

    # Calibrate wrist after moving to position
    ur_handle.calibrate_wrist()

    ur_handle.move_to_position(p2.x, p2.y, p2.z, p2.rx, p2.ry, p2.rz, SPEED, ACCELERATION)
    print("Position 2 Reached")
    time.sleep(1)

    ur_handle.move_to_position(p3.x, p3.y, p3.z, p3.rx, p3.ry, p3.rz, CUT_SPEED, ACCELERATION)
    print("Position 3 Reached")
    time.sleep(1)


if __name__ == '__main__':
    main()
