import os
import numpy as np
from scipy import signal
from scipy.fft import fft, fftfreq
import pandas as pd
import matplotlib
from matplotlib import pyplot as plt

matplotlib.use('Qt5Agg')


def find_peaks(data, prominence=14):
    peaks, _ = signal.find_peaks(data, prominence=prominence)
    return peaks


def find_valleys(data, prominence=14):
    valleys, _ = signal.find_peaks(-data, prominence=prominence)
    return valleys


def detrend_data(data):
    det_data = signal.detrend(data)
    return det_data


def get_mean_distance(pos, peaks):
    # Calculate mean distance between peaks
    pos_peaks = np.asarray([pos[p] for p in peaks])
    dist = np.diff(pos_peaks)
    mean_distance = np.mean(np.abs(dist)) * 1000
    return mean_distance


def select_file():
    filename = '44-2_153148.csv'
    fp = os.path.join(os.getcwd(), 'outputs', filename)
    if not os.path.isfile(fp):
        raise FileNotFoundError("File not found.")
    return fp


def compute_fft(data, sample_rate):
    n = len(data)
    yf = fft(data)
    xf = fftfreq(n, 1 / sample_rate)
    amplitude = 2.0 / n * np.abs(yf[:n // 2])
    return xf[:n // 2], amplitude


def trim_data_fy_spike(fy, threshold=0.2):
    # Identify the start and end indices of the area of interest based on large spikes and drops
    start_index = next((i for i, v in enumerate(fy) if v > threshold), None)
    end_index = next((i for i, v in enumerate(fy[::-1]) if v < -threshold), None)

    if start_index is None or end_index is None:
        print("Area of interest not found based on the threshold.")
        return 0, len(fy)  # Return the full range if not found

    end_index = len(fy) - end_index  # Convert end_index from reversed index to actual index
    print(f'Start index: {start_index}, End index: {end_index}')
    return start_index, end_index


def trim_data_fy_zero(fy, threshold=1, duration=30):
    # Identify the start and end indices of the area of interest
    # Start index: when Fy exceeds the threshold for a duration
    # End index: when Fy returns below the threshold for a duration
    start_index, end_index = None, None
    count_above, count_below = 0, 0

    for i in range(len(fy)):
        if fy[i] > threshold:
            count_above += 1
            count_below = 0
        else:
            count_below += 1
            count_above = 0

        if start_index is None and count_above >= duration:
            start_index = i - duration + 1

        if start_index is not None and count_below >= duration:
            end_index = i - duration + 1
            break

    if start_index is None or end_index is None:
        print("Area of interest not found based on the threshold and duration.")
        return 0, len(fy)  # Return the full range if not found
    print(f'Start index: {start_index}, End index: {end_index}')
    return start_index, end_index


def main():
    file_path = select_file()

    df = pd.read_csv(file_path)

    if df.empty:
        print("df is empty. Please check the .csv file.")
        exit()

    # Check if the DataFrame contains the expected columns
    expected_columns = ['timestamp', 'Fx(N)', 'Fy(N)', 'Fz(N)', 'TRx(Nm)', 'TRy(Nm)', 'TRz(Nm)', 'calibrated_Fx(N)',
                        'calibrated_Fy(N)', 'calibrated_Fz(N)', 'calibrated_TRx(Nm)', 'calibrated_TRy(Nm)',
                        'calibrated_TRz(Nm)', 'x_m', 'y_m', 'z_m', 'rx_rad', 'ry_rad', 'rz_rad']
    if not all(col in df.columns for col in expected_columns):
        print("df is missing expected columns. Please check the .csv file.")
        exit()
    print("df loaded successfully with expected columns:", expected_columns)

    time = df['timestamp'].values
    fy = df['calibrated_Fy(N)'].values
    y_m = df['y_m'].values

    # Trim data to the area of interest
    start_index, end_index = trim_data_fy_zero(fy)

    time = time[start_index:end_index]
    fy = fy[start_index:end_index]
    y_m = y_m[start_index:end_index]

    peaks = find_peaks(fy)

    mean_dist = get_mean_distance(y_m, peaks)
    print(f'Mean distance between peaks: {mean_dist} mm.')

    # Plot Fy with peaks highlighted
    plt.figure()
    plt.plot(time, fy, label='Fy(N)')
    plt.plot(time[peaks], fy[peaks], 'rx', label='Peaks')
    plt.xlabel('Time [s]')
    plt.ylabel('Force Fy(N)')
    plt.title('Fy(N) with Peaks Highlighted')
    plt.legend()
    plt.grid(True)
    plt.show(block=False)

    # FFT Analysis
    sample_rate = 1 / (time[1] - time[0])  # Sample rate based on time intervals
    frequencies, amplitude = compute_fft(fy, sample_rate)

    plt.figure()
    plt.plot(frequencies, amplitude)
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Amplitude')
    plt.title('FFT Amplitude Spectrum of Fy(N)')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()

# # Create time NumPy array, normalize time, identify area of interest - not necessary w/ 07/25/2024 .csv generation
#
# base_X = 0
# base_Y = 0
# base_Z = 0
# start_time = 0
# end_time = 20
#
# time = df.timestamp.values.tolist()
# time = np.array(time)
# print("Original 'time' array:", time)
# df['timestamp'].to_csv('time.txt', index=False, header=True)
# print("Timestamps saved to time.txt")
#
# time = time - time[0]
# print("Modified 'time' array:", time)
# print("Size of 'time' array:", len(time))
#
# start_index = np.argmin(np.abs(time - start_time))
# end_index = np.argmin(np.abs(time - end_time))
#
# print("start_index:", start_index)
# print("end_index:", end_index)
# print("time at start index:", time[start_index])
# print("time at end index:", time[end_index])
#
# time = time[start_index:end_index]
# time = time - time[0]
#
# plots = ['calibrated_Fx(N)', 'calibrated_Fy(N)', 'calibrated_Fz(N)']
#
# for plot in plots:
#     data = np.array(df[plot].tolist())
#     data = data[start_index:end_index]
#     data = data - np.mean(data)
#     fig, ax = plt.subplots()
#     title = plot + ' vs. time'
#     fig.suptitle(title, fontsize=20)
#     plt.plot(time, data, linewidth=2)
#     plt.xlabel('Time [s]', fontsize=18)
#     plt.ylabel(r'Force [N]', fontsize=18)
#     plt.grid(True)
#     plt.autoscale(enable=True, axis='y')
#     plt.show()

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
