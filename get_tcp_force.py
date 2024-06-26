# returns force and torque values in tool coordinate system

# reference variables are saved as pose variable to be used for pose_trans()

# force_T: force values (Fx, Fy, Fz) in tool coordinate system at tcp position as origin
# torque_T: torque values (Tx, Ty, Tz) in tool coordinate system at tcp position as origin
# force_B: force values (Fx, Fy, Fz) in base coordinate system at tcp position as origin
# torque_B: torque values (Tx, Ty, Tz) in base coordinate system at tcp position as origin
# tcp: tcp position and tcp orientation
# rotation_BT: Tool orientation

# mathematic calcluation

# force_B = rotation_BT * force_T
# => force_T = inv(rotation_BT) * force_B


import rtde_control
import rtde_receive
import numpy as np
import gc

rtde_c = rtde_control.RTDEControlInterface("172.28.60.3")
rtde_r = rtde_receive.RTDEReceiveInterface("172.28.60.3")

def get_tcp_force_tool():
    force_torque = get_tcp_force()
    force_B = p[force_torque[0], force_torque[1], force_torque[2], 0, 0, 0]
    torque_B = p[force_torque[3], force_torque[4], force_torque[5], 0, 0, 0]
    tcp = get_actual_tcp_pose()
    rotation_BT = p[0, 0, 0, tcp[3], tcp[4], tcp[5]]
    force_T = pose_trans(pose_inv(rotation_BT), force_B)
    torque_T = pose_trans(pose_inv(rotation_BT), torque_B)
    force_torque_T = p[force_T[0], force_T[1], force_T[2], torque_T[0], torque_T[1], torque_T[2]]
    return force_torque_T


end

#### developed by Universal Robots Global Competence Center