import os
import socket
import struct
import time
import math
import argparse
import statistics
from decimal import Decimal
from mpl_toolkits import mplot3d
from statistics import mean
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import interactive

interactive(True)

# ARCORE_CORRECTION = [460, 210, -260]  # upside down, vertical
ARCORE_CORRECTION = [400, 120, -200]  # horizontal, left side down
ARKIT_CORRECTION = [400, 120, -200]  # horizontal, left side down
# ROVIOLI_CORRECTION =[-360, -200, -250]
# ROVIOLI_CORRECTION =[150, +200, -450]
ROVIOLI_CORRECTION = [450, 330, -150]
ORBSLAM_CORRECTION = [-480, 700, -120]
LSD_CORRECTION = [-250, -130, -600]
CALIBRATION_TIME = 20

ARCORE_SCALE_FACTOR = 1000
ARKIT_SCALE_FACTOR = 1000
ROVIOLI_SCALE_FACTOR = 800
ORBSLAM_SCALE_FACTOR = 350
LSD_SCALE_FACTOR = 6000


class TCPConnection:
    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

        self.robot_positions = []
        self.remote_slam_positions = []
        self.arcore_positions = []

    def connect(self, host, port):
        try:
            self.sock.connect((host, port))
            print('Successful Connection')
        except:
            print('Connection Failed')

    def getdata_from_robot(self):

        # for index see attachment from the bottom: https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/

        temp_list = []
        try:
            packet_1 = self.sock.recv(4)  # Total message length in bytes
            message_length = struct.unpack('!i', packet_1)[0]
            # print("message length: ", struct.unpack('!i', message_length)[0])
            if message_length == 1116:

                packet_2 = self.sock.recv(8)  # Time elapsed since the controller was started
                # print("elapsed time: ", struct.unpack('!d', elapsed_time)[0])
                elapsed_time = struct.unpack('!d', packet_2)[0]
                temp_list.append(elapsed_time)

                packet_3 = self.sock.recv(48)  # Target joint positions
                packet_4 = self.sock.recv(48)  # Target joint velocities
                packet_5 = self.sock.recv(48)  # Target joint accelerations
                packet_6 = self.sock.recv(48)  # Target joint currents
                packet_7 = self.sock.recv(48)  # Target joint moments (torques)
                packet_8 = self.sock.recv(48)  # Actual joint positions
                packet_9 = self.sock.recv(48)  # Actual joint velocities
                packet_10 = self.sock.recv(48)  # Actual joint currents
                packet_11 = self.sock.recv(48)  # Joint control currents

                # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz),
                # where rx, ry and rz is a rotation vector representation of the tool orientation
                packet_12 = self.sock.recv(8)
                x = struct.unpack('!d', packet_12)[0]
                temp_list.append(x * 1000)
                # print("X = ", x * 1000)

                packet_13 = self.sock.recv(8)
                y = struct.unpack('!d', packet_13)[0]
                # print("Y = ", y * 1000)
                temp_list.append(y * 1000)

                packet_14 = self.sock.recv(8)
                z = struct.unpack('!d', packet_14)[0]
                # print("Z = ", z * 1000)
                temp_list.append(z * 1000)

                packet_15 = self.sock.recv(8)
                Rx = struct.unpack('!d', packet_15)[0]
                # print("Rx = ", Rx)
                # temp_list.append(Rx)

                packet_16 = self.sock.recv(8)
                Ry = struct.unpack('!d', packet_16)[0]
                # print("Ry = ", Ry)
                # temp_list.append(Ry)

                packet_17 = self.sock.recv(8)
                Rz = struct.unpack('!d', packet_17)[0]
                # print("Rz = ", Rz)
                # temp_list.append(Rz)

                qx, qy, qz, qw = euler_to_quaternion(Rx, Ry, Rz)
                temp_list.append(qx)
                temp_list.append(qy)
                temp_list.append(qz)
                temp_list.append(qw)

                if len(temp_list) > 0:
                    self.robot_positions.append(temp_list)
                    print(temp_list)

        except Exception as e:
            print("Error: ", e)

    def save_data(self):
        robot_data_filename = 'measurements/robot_' + time.strftime("%Y%m%d_%H%M%S") + '.txt'
        with open(robot_data_filename, "w") as f:
            for listitem in self.robot_positions:
                f.write('%s\n' % listitem)


def getdata_from_remote_slam(path):
    with open(path) as f:
        lines = f.read().splitlines()
    new_list = []
    list = []
    for i in lines:
        list.append(i.split(",("))

    for i in list:
        templ_list = []
        for j in i:
            if ', ' in j:
                templ_list.append(j.replace(")", "").split(', '))
            else:
                templ_list.append(j.split(","))
            new_list.append(templ_list)

    return new_list


def read_ground_truth(path):
    with open(path) as f:
        lines = [line.replace("[", "").replace("]", "").replace("\n", "").replace(" ", "").split(",") for line in f]
    return lines


def read_slam_file(path):
    with open(path) as f:
        lines = [
            line.replace("(", "").replace(")", "").replace("--end--", "").replace("\n", "").replace(" ", "").split(",")
            for line in f]
    return lines


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    res = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                    [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                    [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
    return res


def calculate_rotation_matrix(gt_first_position, gt, slam, time=15):
    print("Calculate rotation matrix...")
    minimum_distance = [1000000000, [0, 0, 0], 0]

    cut_here = -1
    for i in range(0, len(slam)):
        if float(slam[i][0]) > float(slam[0][0]) + time * 1000000000:
            cut_here = i - 1
    slam = slam[:cut_here]

    cut_here = -1
    for i in range(0, len(gt)):
        if float(gt[i][0]) > float(gt[0][0]) + time * 1000000000:
            cut_here = i - 1
    gt = gt[:cut_here]

    gt_x, gt_y, gt_z = [], [], []
    for i in gt:
        if float(i[0]) < float(gt[0][0]) + time:
            gt_x.append(float(i[1]))
            gt_y.append(float(i[2]))
            gt_z.append(float(i[3]))

    # if float(slam[i][0]) / 1000000000 < float(slam[0][0]) / 1000000000 + time:

    for axis in [[1, 0, 0], [0, 1, 0], [0, 0, 1]]:
        for radian in range(0, 471):
            slam_x, slam_y, slam_z = [], [], []
            for i in range(0, len(slam)):
                converted_slam_position = np.dot(
                    rotation_matrix(axis, radian / 100),

                    # x pose convert to mm and subtract gt zero x
                    [(float(slam[i][2]) * 1000 - float(gt_first_position[1])),

                     # z pose convert to mm and subtract gt zero z
                     # change the y and z because robot coordinate system is left-handed,
                     # but the SLAM is right-handed
                     float(slam[i][4]) * 1000 - float(gt_first_position[3]),

                     # y pose convert to mm and subtract gt zero y + constant offset
                     float(slam[i][3]) * 1000 - float(gt_first_position[2])])
                slam_x.append(converted_slam_position[0])
                slam_y.append(converted_slam_position[1])
                slam_z.append(converted_slam_position[2])

            distance = calculate_ATE(gt_x, gt_y, gt_z, slam_x, slam_y, slam_z)

            if distance <= minimum_distance[0]:
                minimum_distance[0] = distance
                minimum_distance[1] = axis
                minimum_distance[2] = radian / 100
                print("distance: ", str(minimum_distance))

    print("minimum distance: " + str(minimum_distance[0]) + " rotation matrix: " + str(minimum_distance[1]) + str(
        minimum_distance[2]))
    return minimum_distance


def analyze(ground_truth, list_of_slam, result_folder, list_of_slam_types, plot_first_x_sec, pdf):
    print("Analyze trajectories...")
    gt = read_ground_truth(ground_truth)

    # slam = read_slam_file(slam_)

    slams_list = []
    for i in list_of_slam:
        # with open(i, "r") as f:
        #     lines = f.readlines()
        # with open(i, "w") as f:
        #     for line in range(0, len(lines)):
        #         if lines[line].strip("\n") == "Start,Stop,Position,Rotation":
        #             if line > 30:
        #                 if lines[line] != "--end--":
        #                     f.write(lines[line])
        #         else:
        #             f.write(lines[line])

        slams_list.append(read_slam_file(i))

    # for i in list_of_slam_types:
    #     slam_types_list.append(read_slam_file(i))

    gt_first_position = gt[0]

    # calculate avg time between frames (fist element of slams_list)
    framerate_list = []
    for i in range(0, len(slams_list[0]) - 1):
        framerate_list.append((float(slams_list[0][i + 1][0]) - float(slams_list[0][i][0])) / 1000000000)
    avg_time_between_frames = sum(framerate_list) / len(framerate_list)

    # convert ground truth into slam frequency
    # TODO: it might not be necessary. Hint: in ATE, RPE calculation check the closest points
    gt_scaled = [gt[0]]
    temp_avg = []
    for i in range(0, len(gt)):
        if float(gt[i][0]) < (float(gt_scaled[-1][0]) + avg_time_between_frames):
            # collect every frame in avg_time_between_frames
            temp_avg.append([float(j) for j in gt[i]])
        else:
            # get avg of temp_avg list and add to gt_new
            if len(temp_avg) > 0:
                in_float = [*map(mean, zip(*temp_avg))]
                gt_scaled.append([str(k) for k in in_float])
            temp_avg = []

    # convert slam coordinates into the robot coordinate system
    list_of_slam_converted = []
    for j in range(0, len(slams_list)):
        # rot_matrix = calculate_rotation_matrix(gt_first_position, gt, slams_list[j], CALIBRATION_TIME)
        list_of_slam_converted.append([])
        for i in range(0, len(slams_list[j])):
            if len(slams_list[j][i]) > 1:
                if "arcore" in list_of_slam_types[j].lower():
                    converted_slam_position = np.dot(
                        rotation_matrix([0, 0, 1],
                                        # -1.65079633),
                                        -1.20079633),

                        # x pose convert to mm and subtract gt zero x + constant offset
                        [(float(slams_list[j][i][2]) * ARCORE_SCALE_FACTOR - float(gt_first_position[1])) +
                         ARCORE_CORRECTION[0],

                         # z pose convert to mm and subtract gt zero z + constant offset
                         # change the y and z because robot coordinate system is left-handed,
                         # but the SLAM is right-handed
                         float(slams_list[j][i][4]) * ARCORE_SCALE_FACTOR - float(gt_first_position[3]) +
                         ARCORE_CORRECTION[1],

                         # y pose convert to mm and subtract gt zero y + constant offset
                         float(slams_list[j][i][3]) * ARCORE_SCALE_FACTOR - float(gt_first_position[2]) +
                         ARCORE_CORRECTION[2]
                         ])

                elif "rovioli" in list_of_slam_types[j].lower():
                    converted_slam_position = np.dot(
                        rotation_matrix([0, 0, 1],
                                        # -1.57079633),
                                        -1.50079633),
                        # -0.93079633),  # -90° in radians
                        # 3.14),  # 180° in radians
                        # 0.0),  # 0° in radians
                        # -4.71),  # -270° in radians

                        # x pose convert to mm and subtract gt zero x + constant offset
                        [(float(slams_list[j][i][2]) * ROVIOLI_SCALE_FACTOR - float(gt_first_position[1]) +
                          ROVIOLI_CORRECTION[0]),

                         # z pose convert to mm and subtract gt zero z + constant offset
                         # change the y and z because robot coordinate system is left-handed,
                         # but the SLAM is right-handed
                         (float(slams_list[j][i][4]) * ROVIOLI_SCALE_FACTOR - float(gt_first_position[3]) +
                          ROVIOLI_CORRECTION[1]),

                         # y pose convert to mm and subtract gt zero y + constant offset
                         (float(slams_list[j][i][3]) * ROVIOLI_SCALE_FACTOR - float(gt_first_position[2]) +
                          ROVIOLI_CORRECTION[2])
                         ])
                elif "orb" in list_of_slam_types[j].lower():
                    converted_slam_position = np.dot(
                        rotation_matrix([0, 0, 1],
                                        # -1.57079633),
                                        -1.50079633),
                        # -0.93079633),  # -90° in radians
                        # 3.14),  # 180° in radians
                        # 0.0),  # 0° in radians
                        # -4.71),  # -270° in radians

                        # x pose convert to mm and subtract gt zero x + constant offset
                        [(float(slams_list[j][i][2]) * ORBSLAM_SCALE_FACTOR - float(gt_first_position[1]) +
                          ORBSLAM_CORRECTION[0]),

                         # z pose convert to mm and subtract gt zero z + constant offset
                         # change the y and z because robot coordinate system is left-handed,
                         # but the SLAM is right-handed
                         (float(slams_list[j][i][4]) * ORBSLAM_SCALE_FACTOR - float(gt_first_position[3]) +
                          ORBSLAM_CORRECTION[1]),

                         # y pose convert to mm and subtract gt zero y + constant offset
                         (float(slams_list[j][i][3]) * ORBSLAM_SCALE_FACTOR - float(gt_first_position[2]) +
                          ORBSLAM_CORRECTION[2])
                         ])
                elif "arkit" in list_of_slam_types[j].lower():
                    converted_slam_position = np.dot(
                        rotation_matrix([0, 0, 1],
                                        # -1.57079633),
                                        -1.50079633),
                        # -0.93079633),  # -90° in radians
                        # 3.14),  # 180° in radians
                        # 0.0),  # 0° in radians
                        # -4.71),  # -270° in radians

                        # x pose convert to mm and subtract gt zero x + constant offset
                        [(float(slams_list[j][i][2]) * ARKIT_SCALE_FACTOR - float(gt_first_position[1]) +
                          ARKIT_CORRECTION[0]),

                         # z pose convert to mm and subtract gt zero z + constant offset
                         # change the y and z because robot coordinate system is left-handed,
                         # but the SLAM is right-handed
                         (float(slams_list[j][i][4]) * ARKIT_SCALE_FACTOR - float(gt_first_position[3]) +
                          ARKIT_CORRECTION[1]),

                         # y pose convert to mm and subtract gt zero y + constant offset
                         (float(slams_list[j][i][3]) * ARKIT_SCALE_FACTOR - float(gt_first_position[2]) +
                          ARKIT_CORRECTION[2])
                         ])
                elif "lsd" in list_of_slam_types[j].lower():
                    converted_slam_position = np.dot(
                        rotation_matrix([0, 1, 0],
                                        -1.97079633),
                        # -1.33079633),  # -90° in radians
                        # 3.14),  # 180° in radians
                        # 0.0),  # 0° in radians
                        # -4.71),  # -270° in radians

                        # x pose convert to mm and subtract gt zero x + constant offset
                        [(float(slams_list[j][i][2]) * LSD_SCALE_FACTOR - float(gt_first_position[1]) + LSD_CORRECTION[
                            0]),

                         # z pose convert to mm and subtract gt zero z + constant offset
                         # change the y and z because robot coordinate system is left-handed,
                         # but the SLAM is right-handed
                         (float(slams_list[j][i][4]) * -LSD_SCALE_FACTOR - float(gt_first_position[3]) + LSD_CORRECTION[
                             1]),

                         # y pose convert to mm and subtract gt zero y + constant offset
                         (float(slams_list[j][i][3]) * -LSD_SCALE_FACTOR - float(gt_first_position[2]) + LSD_CORRECTION[
                             2])
                         ])

                list_of_slam_converted[j].append([
                    # convert start time to sec - slam zero time + ground truth zero time
                    str(float(slams_list[j][i][0]) / 1000000000 - float(slams_list[j][0][0]) / 1000000000 + float(
                        gt_first_position[0])),

                    # convert stop time to sec - slam zero time + ground truth zero time
                    str(float(slams_list[j][i][1]) / 1000000000 - float(slams_list[j][0][0]) / 1000000000 + float(
                        gt_first_position[0])),

                    str(converted_slam_position[0]),
                    str(converted_slam_position[1]),
                    str(converted_slam_position[2]),
                    str(float(slams_list[j][i][5])),  # + float(gt_first_position[4])),
                    str(float(slams_list[j][i][6])),  # + float(gt_first_position[5])),
                    str(float(slams_list[j][i][7])),  # + float(gt_first_position[6])),
                    str(float(slams_list[j][i][8])),  # + float(gt_first_position[7])),

                    # start time (send the camera image and IMU data to the slam)
                    str(float(slams_list[j][i][0])),

                    # end time (when the slam result returns)
                    str(float(slams_list[j][i][1]))
                ])

                # print("\n\n\ngt zero: ", gt_first_position)
                # print("\nslam: ", slam[i])
                # print("\nslam_converted: ", slam_converted[i])

    # save converted gt and slam positions into files
    with open(ground_truth[:-4] + '_converted.txt', "w") as f:
        f.write('\n'.join([' '.join(i) for i in gt_scaled]))
    for i in range(0, len(list_of_slam_converted)):
        with open(list_of_slam[i][:-4] + '_converted.txt', "w") as f:
            f.write('\n'.join([' '.join(i) for i in list_of_slam_converted[i]]))

    # save lists to files into result folder
    if not os.path.exists(result_folder):
        os.makedirs(result_folder)
    with open(os.path.join(result_folder, 'stamped_groundtruth.txt'), "w") as f:
        f.write('\n'.join([' '.join(i) for i in gt_scaled]))
    for i in range(0, len(list_of_slam_converted)):
        filename = 'stamped_traj_estimate_' + str(list_of_slam_types[i]) + "_" + str(i) + ".txt"
        with open(os.path.join(result_folder, filename), "w") as f:
            f.write('\n'.join([' '.join(i) for i in list_of_slam_converted[i]]))

    # save origin slam and gt files into res dir
    with open(os.path.join(result_folder, "origin_gt_file.txt"), "w") as f:
        f.write('\n'.join([' '.join(i) for i in gt]))
    for i in range(0, len(list_of_slam_converted)):
        filename = 'origin_slam_file_' + str(list_of_slam_types[i]) + "_" + str(i) + ".txt"
        with open(os.path.join(result_folder, filename), "w") as f:
            f.write('\n'.join([' '.join(i) for i in slams_list[i]]))

    gt_x, gt_y, gt_z, slam_x, slam_y, slam_z, start_time, stop_time, stats = [], [], [], [], [], [], [], [], []
    for i in gt:
        gt_x.append(float(i[1]))
        gt_y.append(float(i[2]))
        gt_z.append(float(i[3]))
    for j in range(0, len(list_of_slam_converted)):
        slam_x.append([])
        slam_y.append([])
        slam_z.append([])
        start_time.append([])
        stop_time.append([])
        for k in list_of_slam_converted[j]:
            start_time[j].append(float(k[0]))
            stop_time[j].append(float(k[1]))
            slam_x[j].append(float(k[2]))
            slam_y[j].append(float(k[3]))
            slam_z[j].append(float(k[4]))

        # use evaluate_ate and evaluate_rpe instead
        ate = calculate_ATE(gt_x, gt_y, gt_z, slam_x[j], slam_y[j], slam_z[j])
        print("ATE: ", ate)

        avg_delay, stdev, list_of_delay = calculate_delay_stdev(start_time[j], stop_time[j])
        print("Delay: ", avg_delay)
        print("Stdev: ", stdev)

        # ATE, avg_delay, stdev, list_of_delay converted to ms, elapsed time since the first packet was sent
        # stats.append([ate, avg_delay, stdev, [l * 1000 for l in list_of_delay],  [k - start_time[j][0] for k in start_time[j] if (k - start_time[j][0]) < 50]])
        stats.append([ate, avg_delay, stdev, [l * 1000 for l in list_of_delay],
                      [k - start_time[j][0] for k in start_time[j] if (k - start_time[j][0]) < plot_first_x_sec]])

    visualize(gt_x=gt_x, gt_y=gt_y, gt_z=gt_z, slam_x=slam_x, slam_y=slam_y, slam_z=slam_z, result_folder=result_folder,
              list_of_slam_types=list_of_slam_types, pdf=pdf, stats=stats, plot_first_x_sec=plot_first_x_sec)


def calculate_delay_stdev(start_time, stop_time):
    # TODO: calculate jitter
    list_of_delay = []
    for i in range(0, len(start_time)):
        list_of_delay.append(stop_time[i] - start_time[i])

    avg = statistics.mean(list_of_delay)

    stdev = statistics.stdev(list_of_delay)

    return avg, stdev, list_of_delay


def visualize(gt_x, gt_y, gt_z, slam_x, slam_y, slam_z, result_folder, list_of_slam_types, pdf, stats,
              plot_first_x_sec):
    # 3D plot
    fig3d = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter3D(np.array(gt_x), np.array(gt_y), np.array(gt_z), label="Ground truth")
    for i in range(0, len(slam_x)):
        label = str(list_of_slam_types[i]) + "_" + str(i)
        ax.scatter3D(np.array(slam_x[i]), np.array(slam_y[i]), np.array(slam_z[i]), label=label)

    ax.set_xlabel('X [mm]', size=15)
    ax.set_ylabel('Y [mm]', size=15)
    ax.set_zlabel('Z [mm]', size=15)
    ax.legend()
    ax.set(xlim=(-1400, 1000), ylim=(-1400, 1000), zlim=(0, 600))

    # 2D plots
    fig2d, axs_2d = plt.subplots(3, figsize=(7, 13))

    axs_2d[0].plot(np.array(gt_x), np.array(gt_y), linestyle=" ", marker="o", markersize=2, label="Ground truth",
                   color='k')
    axs_2d[1].plot(np.array(gt_x), np.array(gt_z), linestyle=" ", marker="o", markersize=2, label="Ground truth",
                   color='k')
    axs_2d[2].plot(np.array(gt_y), np.array(gt_z), linestyle=" ", marker="o", markersize=2, label="Ground truth",
                   color='k')

    for i in range(0, len(slam_x)):
        label = str(list_of_slam_types[i])
        axs_2d[0].plot(np.array(slam_x[i]), np.array(slam_y[i]), linestyle=" ", marker="o", markersize=1, label=label)
        axs_2d[1].plot(np.array(slam_x[i]), np.array(slam_z[i]), linestyle=" ", marker="o", markersize=1, label=label)
        axs_2d[2].plot(np.array(slam_y[i]), np.array(slam_z[i]), linestyle=" ", marker="o", markersize=1, label=label)

    axs_2d[0].set_xlabel('X [mm]', size=15)
    axs_2d[0].set_ylabel('Y [mm]', size=15)
    axs_2d[0].grid(True)
    axs_2d[0].legend(loc='upper right', markerscale=3)
    axs_2d[0].set(xlim=(-1400, 1400), ylim=(-900, 900))
    # axs_2d[0].set(xlim=(-2400, 2400), ylim=(-1800, 1800))

    axs_2d[1].set_xlabel('X [mm]', size=15)
    axs_2d[1].set_ylabel('Z [mm]', size=15)
    axs_2d[1].grid(True)
    axs_2d[1].legend(loc='upper right', markerscale=3)
    axs_2d[1].set(xlim=(-1400, 1400), ylim=(0, 900))
    # axs_2d[1].set(xlim=(-2400, 2400), ylim=(0, 800))

    axs_2d[2].set_xlabel('Y [mm]', size=15)
    axs_2d[2].set_ylabel('Z [mm]', size=15)
    axs_2d[2].grid(True)
    axs_2d[2].legend(loc='upper right', markerscale=3)
    axs_2d[2].set(xlim=(-1400, 1400), ylim=(0, 900))
    # axs_2d[2].set(xlim=(-2400, 2400), ylim=(0, 800))
    # axs_2d[0].set_title("Real and estimated positions (in the robot coordinate system)")

    # Delay plot
    fig_delay_1, axs_delay_1 = plt.subplots(1, figsize=(6, 5))
    fig_delay_2, axs_delay_2 = plt.subplots(1, figsize=(6, 5))
    # fig_delay, axs_delay = plt.subplots(figsize=(7, 10))

    for i in range(0, len(stats)):
        label = str(list_of_slam_types[i])
        axs_delay_1.plot(np.array(stats[i][4]), np.array(stats[i][3][:len(stats[i][4])]), label=label, markersize=0.1)
    axs_delay_1.set_xlabel('Elapsed time [sec]', size=15)
    axs_delay_1.set_ylabel('End-to-end latency [ms]', size=15)
    axs_delay_1.grid(True)
    axs_delay_1.legend(loc='upper right', markerscale=6)
    axs_delay_1.set_xlim(0, plot_first_x_sec + 2)
    axs_delay_1.set_ylim(0, 700)

    # Delay violin plot
    axs_delay_2.violinplot([x[3] for x in stats])
    axs_delay_2.set_xticklabels(list_of_slam_types, size=15)
    axs_delay_2.set_xticks(np.arange(1, len(list_of_slam_types) + 1))
    axs_delay_2.set_xlim(0.25, len(list_of_slam_types) + 0.75)
    axs_delay_2.set_ylim(0, 700)
    # axs_delay[1].set_xlabel(' ', size=15)
    axs_delay_2.set_ylabel('Latency [ms]', size=15)
    axs_delay_2.grid(True)

    # Create table
    ates = [int(i[0] * 100) / 100 for i in stats]
    delays = [int(i[1] * 100000) / 100 for i in stats]
    stdevs = [int(i[2] * 100000) / 100 for i in stats]

    cell_text = [ates, delays, stdevs]
    rows_header = "ATE", "Avg. delay [ms]", "Stdev. [ms]"
    columns_header = list_of_slam_types

    fig_table, axs_table = plt.subplots(1, figsize=(4, 1.6), tight_layout={'pad': 1})

    axs_table.table(cellText=cell_text,
                    rowLabels=rows_header,
                    colLabels=columns_header,
                    cellLoc='center',
                    loc='center')
    axs_table.get_xaxis().set_visible(False)
    axs_table.get_yaxis().set_visible(False)
    plt.box(on=None)

    plt.show()

    if pdf:
        fig2d.savefig(os.path.join(result_folder, '2D_plots.pdf'))
        fig3d.savefig(os.path.join(result_folder, '3D_plot.pdf'))
        fig_delay_1.savefig(os.path.join(result_folder, 'delay_plot_1.pdf'))
        fig_delay_2.savefig(os.path.join(result_folder, 'delay_plot_2.pdf'))
        fig_table.savefig(os.path.join(result_folder, 'stat_table.pdf'))
    else:
        fig2d.savefig(os.path.join(result_folder, '2D_plots.png'))
        fig3d.savefig(os.path.join(result_folder, '3D_plot.png'))
        fig_delay_1.savefig(os.path.join(result_folder, 'delay_plot_1.png'))
        fig_delay_2.savefig(os.path.join(result_folder, 'delay_plot_2.png'))
        fig_table.savefig(os.path.join(result_folder, 'stat_table.png'))


def calculate_ATE(gt_x, gt_y, gt_z, slam_x, slam_y, slam_z):
    gt_pose = np.array((gt_x[:len(slam_x)], gt_y[:len(slam_y)], gt_z[:len(slam_z)]))
    slam_pose = np.array((slam_x[:len(gt_x)], slam_y[:len(gt_y)], slam_z[:len(gt_z)]))

    dist = np.linalg.norm(gt_pose - slam_pose)
    return dist


def euler_to_quaternion(yaw, roll, pitch):
    # def euler_to_quaternion(pitch, roll, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--capture_robot_positions', action='store_true',
                        help='Capture robot trajectory from IP:30003 port, and generate ground truth')
    parser.add_argument('--analyze', action='store_true',
                        help='analyze slam accuracy, required: ground_truth, slam_result_path, result_folder ')
    parser.add_argument('--gt', action='store', help='ground truth file path')
    parser.add_argument('--slam', action='store', help='list of slam files path')
    parser.add_argument('--slam_type', action='store',
                        help="list of types of SLAM. Options: ARCore, Rovioli, ORBSLAM (ARKit)")
    parser.add_argument('--res', action='store',
                        help='Result folder. Create the given folder and save stamped and converted ground truth, slam files and plots here')
    parser.add_argument('--plot_first_x_sec', action='store',
                        help='Show only the first X sec in the delay plots (Default: 200)', default=200)
    parser.add_argument('--pdf', action='store_true', help="Save plots in pdf format", default=False)

    args = parser.parse_args()
    capture_robot_positions = args.capture_robot_positions
    if capture_robot_positions:
        listen = TCPConnection()
        listen.connect('127.0.0.1', 30003)
        time.sleep(1)

        try:
            while True:
                listen.getdata_from_robot()

                # EndTime = time.time() + (1 / 60)
                # while time.time() - EndTime < 0:
                #     time.sleep(0.001)
        except KeyboardInterrupt:
            listen.save_data()
    else:
        analyze_slam = args.analyze
        ground_truth = args.gt
        slams = args.slam.split(",")
        result_folder = args.res
        plot_first_x_sec = int(args.plot_first_x_sec)
        slams_type = args.slam_type.split(",")

        if len(slams) != len(slams_type):
            print("Sorry! The list of 'slam' and the list of 'slam_type' must be same length!")
            exit()

        for i in slams_type:
            if "arcore" in i.lower() or "rovioli" in i.lower() or "lsd" in i.lower() or "orb" in i.lower() or "arkit" in i.lower():
                print("{}: valid SLAM type".format(str(i)))
            else:
                print("Sorry! This type of SLAM is not implemented yet : ", str(i))
                exit()

        pdf = args.pdf

        if analyze_slam:
            if ground_truth and slams and result_folder:
                analyze(ground_truth=ground_truth, list_of_slam=slams, result_folder=result_folder,
                        list_of_slam_types=slams_type, plot_first_x_sec=plot_first_x_sec, pdf=pdf)
            else:
                print("Please give me ground_truth, slam_result_path and result_folder!")
