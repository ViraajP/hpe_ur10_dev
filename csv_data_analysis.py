import os
import numpy as np
from scipy import signal
import pandas as pd
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib import pyplot as plt


def find_peaks(pos, data):
    # find peaks from data
    
    peaks = signal.find_peaks(data, prominence=4.5)
    return peaks

def detrend_data(pos, data):
    # remove trend from data
    
    det_data = signal.detrend(data)
    return det_data

def get_mean_distance(pos, peaks):
    # calculate mean distance between peaks

    pos_peaks = np.asarray([pos[p] for p in peaks[0]])
    dist = np.diff(pos_peaks)
    mean_distance = np.mean(dist)
    return mean_distance

def select_file():
    filename = 'test_log6.csv'
    fp = os.path.join(os.getcwd(), filename)
    if not os.path.isfile(fp):
        print('File not found.')
        return select_file()
    return fp

file_path = select_file()

df = pd.read_csv(file_path)

if df.empty:
    print("df is empty. Please check the .csv file.")
    exit()

# Check if the DataFrame contains the expected columns
expected_columns = ['timestamp', 'actual_TCP_force_0', 'actual_TCP_force_1', 'actual_TCP_force_2']
if not all(col in df.columns for col in expected_columns):
    print("df is missing expected columns. Please check the .csv file.")
    exit()
print("df loaded successfully with expected columns:", expected_columns)


base_X = 0
base_Y = 0
base_Z = 0
start_time = 0
end_time = 20


time = df.timestamp.values.tolist()
time = np.array(time)
print("Original 'time' array:", time)
df['timestamp'].to_csv('time.txt', index=False, header=True)
print("Timestamps saved to time.txt")

time = time - time[0]
print("Modified 'time' array:", time)
print("Size of 'time' array:", len(time))

start_index = np.argmin(np.abs(time - start_time))
end_index = np.argmin(np.abs(time - end_time))

print("start_index:", start_index)
print("end_index:", end_index)
print("time at start index:", time[start_index])
print("time at end index:", time[end_index])

time = time[start_index:end_index]
time = time - time[0]

plots = [
    'actual_TCP_force_0', 'actual_TCP_force_1', 'actual_TCP_force_2',
    'actual_TCP_force_3', 'actual_TCP_force_4', 'actual_TCP_force_5'
]

for plot in plots:
    data = np.array(df[plot].tolist())
    data = data[start_index:end_index]
    data = data - np.mean(data)
    fig, ax = plt.subplots()
    title = plot + ' vs. time'
    fig.suptitle(title, fontsize=20)
    plt.plot(time, data, linewidth=2)
    plt.xlabel('Time [s]', fontsize=18)
    plt.ylabel(r'Force [N]', fontsize=18)
    plt.grid(True)
    plt.autoscale(enable=True, axis='y')
    plt.show()


# X = np.array(df.actual_TCP_force_0.tolist())
# Y = np.array(df.actual_TCP_force_1.tolist())
# Z = np.array(df.actual_TCP_force_2.tolist())
#
# X = X[start_index:end_index] - base_X
# Y = Y[start_index:end_index] - base_Y
# Z = Z[start_index:end_index] - base_Z
#
# Y_pos = np.array(df.actual_TCP_pose_1.tolist())
# Y_pos = Y_pos[start_index:end_index]
# Y_pos = Y_pos - Y_pos[0]
# Y_pos = Y_pos * -1000
#
# X_pos = np.array(df.actual_TCP_pose_0.tolist())
# X_pos = X_pos[start_index:end_index]
# X_pos = X_pos - Y_pos[0]
# X_pos = X_pos * -1000


# # Plot Force vs. Time
#
# fig, ax = plt.subplots()
# title = 'Force over time X'
# fig.suptitle(title, fontsize=20)
#
# plt.plot(time, X, linewidth=2)
# plt.xlabel('Time [s]', fontsize=18)
# plt.ylabel(r'Force [N]', fontsize=18)
# plt.grid(True)
# plt.autoscale(enable=True, axis='y')
# plt.show()
#
#
# fig, ax = plt.subplots()
# title = 'Force over time Y'
# fig.suptitle(title, fontsize=20)
#
# plt.plot(time, Y, linewidth=2)
#
# plt.xlabel('Time [s]', fontsize=18)
# plt.ylabel(r'Force [N]', fontsize=18)
# plt.grid(True)
# plt.autoscale(enable=True, axis='y', tight=True)
# plt.show()
#
#
# fig, ax = plt.subplots()
# title = 'Force over time Z'
# fig.suptitle(title, fontsize=20)
#
# plt.plot(time, Z, linewidth=2)
#
# plt.xlabel('Time [s]', fontsize=18)
# plt.ylabel(r'Force [N]', fontsize=18)
# plt.grid(True)
# plt.autoscale(enable=True, axis='y', tight=True)
# plt.show()

# Plot Force vs. Position

# fig, ax = plt.subplots()
# title = 'Force over time Y'
# fig.suptitle(title, fontsize=20)
#
# plt.plot(Y_pos, Y, linewidth=5)
#
# plt.xlabel('distance [mm]', fontsize=18)
# plt.ylabel(r'Force [N]', fontsize=18)
# plt.grid(True)
# plt.autoscale(enable=True, axis='y', tight=True)
# plt.show()
#

# start_cut = 17
# end_cut = 232
#
# start_index = np.argmin(np.abs(Y_pos - start_cut))
# end_index = np.argmin(np.abs(Y_pos - end_cut))
#
# Y_cut = Y[start_index:end_index]
# Y_pos_cut = Y_pos[start_index:end_index] - Y_pos[start_index]
#
# det_data = detrend_data(Y_pos_cut, Y_cut)
# peaks = find_peaks(det_data, Y_cut)
# mean_distance = get_mean_distance(Y_pos_cut, peaks)
#
# fig, ax = plt.subplots()
# title = 'Force over time Y'
# fig.suptitle(title, fontsize=20)
# [plt.axvline(Y_pos_cut[p], c='C2', linewidth=2) for p in peaks[0]]
# plt.plot(Y_pos_cut, Y_cut, linewidth=5)
#
# plt.xlabel('distance [mm]', fontsize=18)
# plt.ylabel(r'Force [N]', fontsize=18)
# plt.grid(True)
# plt.autoscale(enable=True, axis='y', tight=True)
# plt.show()
#
#
# fig, ax = plt.subplots()
# title = 'Force over time X'
# fig.suptitle(title, fontsize=20)
#
# plt.plot(X_pos - X_pos[0], X, linewidth=5)
# plt.xlabel('distance [mm]', fontsize=18)
# plt.ylabel(r'Force [N]', fontsize=18)
# plt.grid(True)
# plt.autoscale(enable=True, axis='y', tight=True)
# plt.show()
