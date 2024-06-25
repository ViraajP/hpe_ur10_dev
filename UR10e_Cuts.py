# import packages
import rtde_control
import rtde_receive
import numpy as np
import gc
import argparse

# UR10e 17.28.60.3
# URSIM 192.168.56.101

IP_ADDRESS = "192.168.56.101"
SPEED = 0.5
ACCELERATION = 1.2


class UrHandler:

    def __init__(self):
        self.receive_handler = rtde_receive.RTDEReceiveInterface(IP_ADDRESS)
        self.control_handler = rtde_control.RTDEControlInterface(IP_ADDRESS)
        self.orientations = [[np.pi, 0, 0],
                             [2.261, -2.182, 0],
                             [0, np.pi, 0],
                             [2.182, 2.261, 0]]

    def get_position(self):
        x_m, y_m, z_m, rx_rad, ry_rad, rz_rad = self.receive_handler.getActualTCPPose()
        return x_m, y_m, z_m, rx_rad, ry_rad, rz_rad

    def move_to_position(self, x_m, y_m, z_m, rx_rad, ry_rad, rz_rad):
        self.control_handler.moveL([x_m, y_m, z_m, rx_rad, ry_rad, rz_rad], SPEED, ACCELERATION)

    def f_move_1d(self, direction=None, force_threshold=10, step=0.001):
        direction = direction.lower()
        assert direction in ['x', 'y', 'z'], "Direction must be one of 'x', 'y', 'z'"
        direction_index = {'x': 0, 'y': 1, 'z': 2}[direction]
        while True:
            force = self.receive_handler.getActualTCPForce()
            if abs(force[direction_index]) > force_threshold:
                break
            current_position = list(self.get_position())
            current_position[direction_index] -= step
            self.move_to_position(*current_position[:3], *current_position[3:])

    def f_move_2d(self, move_direction, move_distance, force_direction, force_magnitude):
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

        # Apply force mode and move to target position
        self.control_handler.forceMode(current_position[:3], [0, 0, 0], force_vector, selection_vector, 2)
        self.control_handler.moveL(target_position, SPEED, ACCELERATION)
        self.control_handler.forceModeEnd()


class Position:

    def __init__(self, x, y, z, rx, ry, rz):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz


def main() -> None:
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

    # Parse arguments
    args = parser.parse_args()

    ur_handle = UrHandler()

    if args.command == "get_position":
        position = ur_handle.get_position()
        print(f"Current position: {position}")

    elif args.command == "move_to_position":
        ur_handle.move_to_position(args.x, args.y, args.z, args.rx, args.ry, args.rz)

    else:
        # Default behavior: execute the main sequence
        x_m, y_m, z_m, rx_rad, ry_rad, rz_rad = ur_handle.get_position()

    p1 = Position(0.7512134736981457, 0.02396203487926281, 0.13659579391489712, -2.2601664151354632,
                  2.1811951632376796, 4.014359232979087e-07)

    ur_handle.move_to_position(p1.x, p1.y, p1.z, p1.rx, p1.ry, p1.rz)

    # Move down until contact
    ur_handle.f_move_1d(direction='z', force_threshold=10, step=0.001)

    # Move in -x direction for 100 mm while applying 10N in z
    ur_handle.f_move_2d(move_direction='x', move_distance=-0.1, force_direction='z', force_magnitude=10)



if __name__ == '__main__':
    main()
