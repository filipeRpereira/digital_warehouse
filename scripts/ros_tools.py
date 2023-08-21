#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, Imu
import matplotlib.pyplot as plt
import message_filters
import json
import numpy as np
import argparse

import sys
from datetime import datetime


def read_json_data(json_file):
    # Opening JSON file
    with open(json_file) as file:
        jsonData = json.load(file)

    return jsonData


def get_valid_frames(jsonData):
    valid_seq = []

    for i in range(1, len(jsonData["frames"]) - 1):
        previous_stamp = datetime.fromtimestamp(int(jsonData["frames"][i-1]["header"]["stamp"])/1e9)
        previous_stamp = previous_stamp.strftime('%H:%M:%S.%f')

        actual_stamp = datetime.fromtimestamp(int(jsonData["frames"][i]["header"]["stamp"])/1e9)
        actual_stamp = actual_stamp.strftime('%H:%M:%S.%f')

        if actual_stamp[9] != previous_stamp[9]:
            valid_seq.append(jsonData["frames"][i]["header"]["seq"])

    return valid_seq


def total_effort_per_joint(jsonData, valid_seq):
    total_frames = len(jsonData["frames"])
    effort = np.empty(shape=(len(listOfJoints)))
    effort.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(0, total_frames):
            if jsonData["frames"][j]["header"]["seq"] in valid_seq:
                effort[i] += abs(jsonData["frames"][j][listOfJoints[i]]["effort"])

    return effort


def get_cycle_time(jsonData):
    init_stamp = datetime.fromtimestamp(int(jsonData["frames"][0]["header"]["stamp"]) / 1e9)
    end_stamp = datetime.fromtimestamp(int(jsonData["frames"][len(jsonData["frames"])-1]["header"]["stamp"]) / 1e9)

    start_time = str(init_stamp.time())
    end_time = str(end_stamp.time())

    # convert time string to datetime
    t1 = datetime.strptime(start_time, "%H:%M:%S.%f")
    t2 = datetime.strptime(end_time, "%H:%M:%S.%f")
    delta = t2 - t1

    return delta.total_seconds()


def get_effort_array(jsonData, valid_seq):
    total_frames = len(jsonData["frames"])
    data_array = np.empty(shape=(len(listOfJoints), len(valid_seq)))
    data_array.fill(0)

    for i in range(len(listOfJoints)):
        aux = 0
        for j in range(0, total_frames):
            if jsonData["frames"][j]["header"]["seq"] in valid_seq:
                data_array[i][aux] = jsonData["frames"][j][listOfJoints[i]]["effort"]
                aux += 1

    return data_array


def get_angular_acc_array(jsonData, valid_frames):
    total_frames = len(jsonData["frames"])
    data_array = np.empty(shape=(len(listOfJoints), len(valid_frames)))
    data_array.fill(0)

    for i in range(len(listOfJoints)):
        aux = 0
        for j in range(0, total_frames):
            if jsonData["frames"][j]["header"]["seq"] in valid_frames:
                idx_valid_frames = valid_frames.index(jsonData["frames"][j]["header"]["seq"])

                next_idx_value_valid_frames = idx_valid_frames + 1

                if next_idx_value_valid_frames < len(valid_frames):
                    valid_frames_next_frame = valid_frames[next_idx_value_valid_frames]
                else:
                    valid_frames_next_frame = len(valid_frames)

                velocity_now = jsonData["frames"][j][listOfJoints[i]]["velocity"]

                velocity_next = 0
                for k in range(0, total_frames):
                    if jsonData["frames"][k]["header"]["seq"] == valid_frames_next_frame:
                        velocity_next = jsonData["frames"][k][listOfJoints[i]]["velocity"]

                data_array[i][aux] = (velocity_next - velocity_now) / 0.1
                aux += 1

    return data_array


def get_valid_timestamp_frame(jsonData, valid_frames):
    total_frames = len(jsonData["frames"])
    timestamp_array = []

    for k in range(0, total_frames):
        if jsonData["frames"][k]["header"]["seq"] == valid_frames[0]:
            first_timestamp = datetime.fromtimestamp(int(jsonData["frames"][k]["header"]["stamp"]) / 1e9)

    start_time = str(first_timestamp.time())

    t1 = datetime.strptime(start_time, "%H:%M:%S.%f")

    for i in range(0, total_frames):
        if jsonData["frames"][i]["header"]["seq"] in valid_frames:
            actual_stamp = datetime.fromtimestamp(int(jsonData["frames"][i]["header"]["stamp"]) / 1e9)
            end_time = str(actual_stamp.time())
            t2 = datetime.strptime(end_time, "%H:%M:%S.%f")

            delta = t2 - t1
            timestamp_array.append(round(delta.total_seconds(), 2))

    return timestamp_array


def plot_effort_sim(_effort_array_0, _effort_array_1, _effort_array_2,
                    _timestamp_array_0, _timestamp_array_1, _timestamp_array_2,
                    joint_num):
    plt.title("Effort - " + listOfJoints[joint_num])
    plt.xlabel("Duration (s)")
    plt.ylabel("Torque (Nm)")

    plt.plot(_timestamp_array_0, _effort_array_0[joint_num])
    plt.plot(_timestamp_array_1, _effort_array_1[joint_num])
    #plt.plot(_timestamp_array_2, _effort_array_2[joint_num])

    plt.xticks(np.linspace(0.0, _timestamp_array_0[-1], num=10))
    plt.xticks(rotation=90)

    #plt.legend(["Manual Programming", "RL with optimization", "RL with optimization (drag)"], loc="lower right")
    plt.legend(["Manual Programming", "RL with optimization"], loc="lower right")

    plt.show()


def plot_angular_acc(_angular_acc_array_0, _angular_acc_array_1, _angular_acc_array_2,
                         _timestamp_array_0, _timestamp_array_1, _timestamp_array_2, joint_num):
    plt.title("Angular acceleration - " + listOfJoints[joint_num])
    plt.xlabel("Duration (s)")
    plt.ylabel("Angular Acceleration (rad/s²)")

    plt.plot(_timestamp_array_0, _angular_acc_array_0[joint_num])
    plt.plot(_timestamp_array_1, _angular_acc_array_1[joint_num])
    #plt.plot(_timestamp_array_2, _angular_acc_array_2[joint_num])

    plt.xticks(np.linspace(0.0, _timestamp_array_0[-1], num=10))
    plt.xticks(rotation=90)

    #plt.legend(["Manual Programming", "RL with optimization", "RL with optimization (drag)"], loc="lower right")
    plt.legend(["Manual Programming", "RL with optimization"], loc="lower right")

    plt.show()


def get_valid_frames_sim(jsonData):
    valid_seq = []

    for i in range(1, len(jsonData["frames"]) - 1):
        previous_stamp = jsonData["frames"][i - 1]["header"]["stamp"]
        actual_stamp = jsonData["frames"][i]["header"]["stamp"]
        if len(previous_stamp) == 9:
            _previous_stamp = '0.' + previous_stamp

        elif len(previous_stamp) == 10:
            _previous_stamp = previous_stamp[0] + '.' + previous_stamp[1:]

        elif len(previous_stamp) == 11:
            _previous_stamp = previous_stamp[0:2] + '.' + previous_stamp[2:]

        if len(actual_stamp) == 9:
            _actual_stamp = '0.' + actual_stamp
        elif len(actual_stamp) == 10:
            _actual_stamp = actual_stamp[0] + '.' + actual_stamp[1:]
        elif len(actual_stamp) > 10:
            _actual_stamp = actual_stamp[0:2] + '.' + actual_stamp[2:]

        if _actual_stamp[1] == '.':
            indx_atual = 1
        elif _actual_stamp[2] == '.':
            indx_atual = 2

        if _previous_stamp[1] == '.':
            indx_previous = 1
        elif _previous_stamp[2] == '.':
            indx_previous = 2

        if _actual_stamp[indx_atual+1] != _previous_stamp[indx_previous+1]:
            valid_seq.append(jsonData["frames"][i]["header"]["seq"])

    return valid_seq


def get_valid_timestamp_frame_sim(jsonData, valid_frames):
    total_frames = len(jsonData["frames"])
    timestamp_array = []

    for i in range(0, total_frames):
        if jsonData["frames"][i]["header"]["seq"] in valid_frames:
            timestamp_array.append(jsonData["frames"][i]["header"]["stamp"])
            # print(jsonData["frames"][i]["header"]["stamp"])

    for i in range(len(timestamp_array)):
        if len(timestamp_array[i]) == 9:
            timestamp_array[i] = round(float('0.' + timestamp_array[i]), 2)
        elif len(timestamp_array[i]) == 10:
            timestamp_array[i] = round(float(timestamp_array[i][0] + '.' + timestamp_array[i][1:]), 2)
        elif len(timestamp_array[i]) == 11:
            timestamp_array[i] = round(float(timestamp_array[i][0:2] + '.' + timestamp_array[i][2:]), 2)

    return timestamp_array


if __name__ == '__main__':
    # Define and parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name_0', help='Save data from ROS topics.',
                        required=False, default="phase_0_27")
    parser.add_argument('--task_name_1', help='Save data from ROS topics.',
                        required=False, default="phase_2_27")
    parser.add_argument('--task_name_2', help='Save data from ROS topics.',
                        required=False, default="phase_2_27")
    parser.add_argument('--read_data', help='Read data from json file.',
                        required=False, default=False)
    parser.add_argument('--plot_acc', help='Plot the angular acceleration of each individual joint.',
                        required=False, default=False)
    parser.add_argument('--joint_num', help='Name of the joint to plot the angular acceleration for 3 samples.',
                        required=False, default="1")
    parser.add_argument('--plot_effort', help='Plot the effort of each individual joint.',
                        required=False, default=False)
    parser.add_argument('--plot_position', help='Plot the position of each individual joint',
                        required=False, default=False)

    args = parser.parse_args()

    json_file_0 = "/home/filipe/Desktop/Dissertação/" + args.task_name_0 + ".json"
    json_file_1 = "/home/filipe/Desktop/Dissertação/" + args.task_name_1 + ".json"
    json_file_2 = "/home/filipe/Desktop/Dissertação/" + args.task_name_2 + ".json"

    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    if args.read_data:
        jsonData_0 = read_json_data(json_file_0)
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)

        valid_frames_0 = get_valid_frames_sim(jsonData_0)
        effort_joint_0 = total_effort_per_joint(jsonData_0, valid_frames_0)

        valid_frames_1 = get_valid_frames(jsonData_1)
        effort_joint_1 = total_effort_per_joint(jsonData_1, valid_frames_1)

        valid_frames_2 = get_valid_frames(jsonData_2)
        effort_joint_2 = total_effort_per_joint(jsonData_2, valid_frames_2)


        cycle_time_0 = get_cycle_time(jsonData_0)
        print("Total effort 0 : " + str(round(sum(effort_joint_0))) + " Nm")
        print("Cycle time   0 : " + str(round(float(cycle_time_0), 2)) + " s")
        print("---------------------------------")
        cycle_time_1 = get_cycle_time(jsonData_1)
        print("Total effort 1 : " + str(round(sum(effort_joint_1))) + " Nm")
        print("Cycle time   1 : " + str(round(float(cycle_time_1), 2)) + " s")
        print("---------------------------------")
        cycle_time_2 = get_cycle_time(jsonData_2)
        print("Total effort 2 : " + str(round(sum(effort_joint_2))) + " Nm")
        print("Cycle time   2 : " + str(round(float(cycle_time_2), 2)) + " s")

    if args.plot_acc:
        jsonData_0 = read_json_data(json_file_0)
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)

        valid_frames_0 = get_valid_frames_sim(jsonData_0)
        valid_frames_1 = get_valid_frames(jsonData_1)
        valid_frames_2 = get_valid_frames(jsonData_2)

        angular_acc_array_0 = get_angular_acc_array(jsonData_0, valid_frames_0)
        timestamp_array_0 = get_valid_timestamp_frame_sim(jsonData_0, valid_frames_0)

        angular_acc_array_1 = get_angular_acc_array(jsonData_1, valid_frames_1)
        timestamp_array_1 = get_valid_timestamp_frame(jsonData_1, valid_frames_1)

        angular_acc_array_2 = get_angular_acc_array(jsonData_2, valid_frames_2)
        timestamp_array_2 = get_valid_timestamp_frame(jsonData_2, valid_frames_2)

        plot_angular_acc(angular_acc_array_0, angular_acc_array_1, angular_acc_array_2,
                             timestamp_array_0, timestamp_array_1, timestamp_array_2,
                             int(args.joint_num))

    if args.plot_effort:
        jsonData_0 = read_json_data(json_file_0)
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)

        valid_frames_0 = get_valid_frames_sim(jsonData_0)
        valid_frames_1 = get_valid_frames(jsonData_1)
        valid_frames_2 = get_valid_frames(jsonData_2)

        effort_array_0 = get_effort_array(jsonData_0, valid_frames_0)
        effort_array_1 = get_effort_array(jsonData_1, valid_frames_1)
        effort_array_2 = get_effort_array(jsonData_2, valid_frames_2)

        timestamp_array_0 = get_valid_timestamp_frame_sim(jsonData_0, valid_frames_0)
        timestamp_array_1 = get_valid_timestamp_frame(jsonData_1, valid_frames_1)
        timestamp_array_2 = get_valid_timestamp_frame(jsonData_2, valid_frames_2)

        plot_effort_sim(effort_array_0, effort_array_1, effort_array_2,
                        timestamp_array_0, timestamp_array_1, timestamp_array_2,
                        int(args.joint_num))















