import rtde_control
import rtde_receive
import numpy as np
import gc

IP_ADDRESS = "172.28.60.3"
SPEED = 0.5
ACCELERATION = 1.2


class UrHandler:
    """UR Handler."""

    def __init__(self):
        self.receive_handler = rtde_receive.RTDEReceiveInterface(IP_ADDRESS)
        self.control_handler = rtde_control.RTDEControlInterface(IP_ADDRESS)
        self.orientations = [[np.pi, 0, 0], [2.261, -2.182, 0], [0, np.pi, 0], [2.182, 2.261, 0]]

    def get_position(self):
        x_m, y_m, z_m, rx_rad, ry_rad, rz_rad = self.receive_handler.getActualTCPPose()

        return x_m, y_m, z_m, rx_rad, ry_rad, rz_rad

    def move_to_position(self, x_m, y_m, z_m, rx_rad, ry_rad, rz_rad):
        self.control_handler.moveL([x_m, y_m, z_m, rx_rad, ry_rad, rz_rad], SPEED, ACCELERATION)

    def move_to_xyz_position(self, x_m, y_m, z_m):
        _, _, _, rx_rad, ry_rad, rz_rad = self.get_position()
        self.move_to_position(x_m, y_m, z_m, rx_rad, ry_rad, rz_rad)

    def move_to_corner(self, corner):
        self.move_to_xyz_position(corner.x, corner.y, corner.z)

    def orient_to_corner(self, orientation):
        x, y, z, _, _, _ = self.get_position()

        self.move_to_position(x, y, z, orientation[0], orientation[1], orientation[2])