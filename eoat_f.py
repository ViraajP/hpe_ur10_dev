import numpy as np
from pytransform3d import rotations as pr
import os
import pandas as pd
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib import pyplot as plt
from pytransform3d.rotations import matrix_from_euler


def select_file():
    filename = 'test_log6.csv'
    fp = os.path.join(os.getcwd(), filename)
    if not os.path.isfile(fp):
        print('File not found.')
        return select_file()
    return fp

file_path = select_file()

df = pd.read_csv(file_path)

forces = [
    'actual_TCP_force_0', 'actual_TCP_force_1', 'actual_TCP_force_2',
    'actual_TCP_force_3', 'actual_TCP_force_4', 'actual_TCP_force_5'
]
torques = [
    'target_moment_0', 'target_moment_1', 'target_moment_2',
    'target_moment_3', 'target_moment_4', 'target_moment_5'
]
positions = ['actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']

data = df[forces + torques + positions].values

# output to .csv to confirm
# pd.DataFrame(data, columns=forces + torques + positions).to_csv('data.csv', index=False, header=True)


def forwardKinematics(theta, tcp=None):
    # theta are the joint angles in radians
    # tcp is the tcp offset as a pose (x,y,z,rx,ry,rz)

    # values for UR-10 (https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/):
    a = np.array([0.0000, -0.612, -0.5723, 0.0000, 0.0000, 0.0000])
    d = np.array([0.1273, 0.0000, 0.0000, 0.163941, 0.1157, 0.0922])
    alpha = np.array([np.pi / 2, 0., 0., np.pi / 2, -np.pi / 2, 0.])

    # values from calibration.conf:
    delta_a = np.array([3.1576640107943976e-05, 0.298634925475782076, 0.227031257526500829, -8.27068507303316573e-05,
                        3.6195435783833642e-05, 0])
    delta_d = np.array([5.82932048768247668e-05, 362.998939868892023, -614.839459588742898, 251.84113332747981,
                        0.000164511802564715204, -0.000899906496469232708])
    delta_alpha = np.array(
        [-0.000774756642435869836, 0.00144883356002286951, -0.00181081418698111852, 0.00068792563586761446,
         0.000450856239573305118, 0])
    delta_theta = np.array([1.09391516130152855e-07, 1.03245736607748673, 6.17452995676434124, -0.92380698472218048,
                            6.42771759845617296e-07, -3.18941184192234051e-08])

    a += delta_a
    d += delta_d
    alpha += delta_alpha
    theta = theta.copy() + delta_theta

    ot = np.eye(4)
    for i in range(6):
        ot = ot @ np.array([[np.cos(theta[i]), -(np.sin(theta[i])) * np.cos(alpha[i]),
                             np.sin(theta[i]) * np.sin(alpha[i]), a[i] * np.cos(theta[i])],
                            [np.sin(theta[i]), np.cos(theta[i]) * np.cos(alpha[i]),
                             -(np.cos(theta[i])) * np.sin(alpha[i]), a[i] * np.sin(theta[i])],
                            [0.0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]], [0.0, 0.0, 0.0, 1.0]])

    offset = np.array([ot[0, 3], ot[1, 3], ot[2, 3]]) + (ot[:3, :3] @ tcp[:3])
    newAngle = pr.compact_axis_angle_from_matrix(ot[:3, :3] @ pr.matrix_from_compact_axis_angle(tcp[3:]))
    result = np.array([offset[0], offset[1], offset[2], newAngle[0], newAngle[1], newAngle[2]])
    return result

# def force_torque():
#     forces = data[:, :6]
#     torques = data[:, 6:12]


def main():
    results = []
    for i in data:
        theta = i[-6:]
        tcp = [0, 0, 0, 0, 0, 0]
        result = forwardKinematics(theta, tcp)
        results.append(result)


    with open('kin_results.csv', 'w', newline='') as f:
        f.write('x,y,z,rx,ry,rz\n')
        for i in results:
            f.write(','.join(map(str, i)) + '\n')


    # Extract X, Y, Z coordinates and Rotation X, Y, Z angles from results
    x = [result[0] for result in results]
    y = [result[1] for result in results]
    z = [result[2] for result in results]
    rot_x = [result[3] for result in results]
    rot_y = [result[4] for result in results]
    rot_z = [result[5] for result in results]

    colors = np.linspace(0, 1, len(results))

    # Plot results in 3D space
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(len(results)):
        ax.scatter(x, y, z, c=colors, cmap='viridis')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Forward Kinematics Results')
    plt.show()

    # rotation_matrices = []
    # for result in results:
    #     rx, ry, rz = result[3:6]
    #     rotation_matrix = matrix_from_euler([rx, ry, rz], 0, 1, 2, extrinsic=False)
    #     rotation_matrices.append(rotation_matrix)
    #
    # # Plotting w/ EoAT angle represented
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    #
    # for result, rotation_matrix in zip(results, rotation_matrices):
    #     x, y, z, _, _, _ = result
    #     axes_length = 0
    #     x_axis = axes_length * np.array([1, 0, 0])
    #     y_axis = axes_length * np.array([0, 1, 0])
    #     z_axis = axes_length * np.array([0, 0, 1])
    #
    #     # Rotate axes vectors
    #     x_axis_rotated = np.dot(rotation_matrix, x_axis)
    #     y_axis_rotated = np.dot(rotation_matrix, y_axis)
    #     z_axis_rotated = np.dot(rotation_matrix, z_axis)
    #
    #     # Plot axes lines
    #     ax.plot([x, x + x_axis_rotated[0]], [y, y + x_axis_rotated[1]], [z, z + x_axis_rotated[2]], color='r')
    #     ax.plot([x, x + y_axis_rotated[0]], [y, y + y_axis_rotated[1]], [z, z + y_axis_rotated[2]], color='g')
    #     ax.plot([x, x + z_axis_rotated[0]], [y, y + z_axis_rotated[1]], [z, z + z_axis_rotated[2]], color='b')
    #
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.set_title('Forward Kinematics Results with Rotation Axes')
    # plt.show()


if __name__ == "__main__":
    main()
