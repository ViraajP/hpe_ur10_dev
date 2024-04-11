from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_control import RTDEControlInterface as RTDEControl
import numpy as np

IP_ADDRESS = "172.28.60.3"


# actual_q = rtde_r.getActualQ()
# print(actual_q)
# rtde_c.moveL([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04], 0.5, 0.3)


def get_tcp_force_tool(rtde_r, rtde_c):
    force_torque = rtde_r.getActualTCPForce()
    force_b = [force_torque[0], force_torque[1], force_torque[2], 0, 0, 0]
    torque_b = [force_torque[3], force_torque[4], force_torque[5], 0, 0, 0]
    tcp = rtde_r.getActualTCPPose()
    rotation_bt = [0, 0, 0, tcp[3], tcp[4], tcp[5]]
    force_t = rtde_c.poseTrans(rotation_bt, force_b)
    torque_t = rtde_c.poseTrans(rotation_bt, torque_b)
    force_torque_t = force_t[:3] + torque_t[:3]
    return force_torque_t


def main():
    rtde_r = RTDEReceive(IP_ADDRESS)
    rtde_c = RTDEControl(IP_ADDRESS)

    force_torque_T = get_tcp_force_tool(rtde_r, rtde_c)
    force_torque = rtde_r.getActualTCPForce()

    print("Force and torque @ base co-ord: ", force_torque)
    print("Force and torque @ tool co-ord:", force_torque_T)


if __name__ == "__main__":
    main()
