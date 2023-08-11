#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, Imu
import matplotlib.pyplot as plt
import message_filters
import json
import numpy as np
import argparse

import sys

time_test = 20000
time_started = 0

rospy.init_node('listener_new', anonymous=False)
jointStates_sub = message_filters.Subscriber('joint_states', JointState)


def callback_check_home_end_position(joints):
    joint_position = joints.position
    home_position = str(joint_position).split(',')

    for i in range(len(home_position)):
        home_position[i] = home_position[i].replace('(', '').replace(')', '')

    home_values = [0.1037, -0.5213, 0.0936, -2.8438, 0.0471, 2.6852, 0.8157, 0.04, 0.04]
    dif_robot = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    dif = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    for i in range(len(home_position)):
        dif_robot[i] = float(home_position[i])

    for i in range(len(dif_robot)):
        dif[i] = abs(dif_robot[i] - home_values[i])

    home = True
    for i in range(len(dif)):
        if dif[i] > 0.1:
            home = False

    if not home:
        #print("Not Home Position", flush=True, end="\r")
        save_json_data(joints)

    else:
        print("Saving ROS data...", flush=True, end="\r")


def listener_ros_topics():
    json_object = json.dumps({
        "frames": []
    }, indent=2)

    with open(json_file_1, "w") as outfile:
        outfile.write(json_object)

    ts = message_filters.TimeSynchronizer([jointStates_sub], 10)
    ts.registerCallback(callback_check_home_end_position)
    rospy.spin()


def save_json_data(data):
    joint_position_0 = data.position[0]
    joint_velocity_0 = data.velocity[0]
    joint_effort_0 = data.effort[0]
    if str(joint_position_0) == "nan":
        joint_position_0 = 0
    if str(joint_velocity_0) == "nan":
        joint_velocity_0 = 0
    if str(joint_effort_0) == "nan":
        joint_effort_0 = 0

    joint_position_1 = data.position[1]
    joint_velocity_1 = data.velocity[1]
    joint_effort_1 = data.effort[1]
    if str(joint_position_1) == "nan":
        joint_position_1 = 0
    if str(joint_velocity_1) == "nan":
        joint_velocity_1 = 0
    if str(joint_effort_1) == "nan":
        joint_effort_1 = 0

    joint_position_2 = data.position[2]
    joint_velocity_2 = data.velocity[2]
    joint_effort_2 = data.effort[2]
    if str(joint_position_2) == "nan":
        joint_position_2 = 0
    if str(joint_velocity_2) == "nan":
        joint_velocity_2 = 0
    if str(joint_effort_2) == "nan":
        joint_effort_2 = 0

    joint_position_3 = data.position[3]
    joint_velocity_3 = data.velocity[3]
    joint_effort_3 = data.effort[3]
    if str(joint_position_3) == "nan":
        joint_position_3 = 0
    if str(joint_velocity_3) == "nan":
        joint_velocity_3 = 0
    if str(joint_effort_3) == "nan":
        joint_effort_3 = 0

    joint_position_4 = data.position[4]
    joint_velocity_4 = data.velocity[4]
    joint_effort_4 = data.effort[4]
    if str(joint_position_4) == "nan":
        joint_position_4 = 0
    if str(joint_velocity_4) == "nan":
        joint_velocity_4 = 0
    if str(joint_effort_4) == "nan":
        joint_effort_4 = 0

    joint_position_5 = data.position[5]
    joint_velocity_5 = data.velocity[5]
    joint_effort_5 = data.effort[5]
    if str(joint_position_5) == "nan":
        joint_position_5 = 0
    if str(joint_velocity_5) == "nan":
        joint_velocity_5 = 0
    if str(joint_effort_5) == "nan":
        joint_effort_5 = 0

    joint_position_6 = data.position[6]
    joint_velocity_6 = data.velocity[6]
    joint_effort_6 = data.effort[6]
    if str(joint_position_6) == "nan":
        joint_position_6 = 0
    if str(joint_velocity_6) == "nan":
        joint_velocity_6 = 0
    if str(joint_effort_6) == "nan":
        joint_effort_6 = 0

    joint_position_7 = data.position[7]
    joint_velocity_7 = data.velocity[7]
    joint_effort_7 = data.effort[7]
    if str(joint_position_7) == "nan":
        joint_position_7 = 0
    if str(joint_velocity_7) == "nan":
        joint_velocity_7 = 0
    if str(joint_effort_7) == "nan":
        joint_effort_7 = 0

    joint_position_8 = data.position[8]
    joint_velocity_8 = data.velocity[8]
    joint_effort_8 = data.effort[8]
    if str(joint_position_8) == "nan":
        joint_position_8 = 0
    if str(joint_velocity_8) == "nan":
        joint_velocity_8 = 0
    if str(joint_effort_8) == "nan":
        joint_effort_8 = 0

    entry = {
        "header": {
            "seq": str(data.header.seq),
            "stamp": str(data.header.stamp)
        },
        "joint_0": {
            "position": joint_position_0,
            "effort": joint_effort_0,
            "velocity": joint_velocity_0,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_1": {
            "position": joint_position_1,
            "effort": joint_effort_1,
            "velocity": joint_velocity_1,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_2": {
            "position": joint_position_2,
            "effort": joint_effort_2,
            "velocity": joint_velocity_2,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_3": {
            "position": joint_position_3,
            "effort": joint_effort_3,
            "velocity": joint_velocity_3,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_4": {
            "position": joint_position_4,
            "effort": joint_effort_4,
            "velocity": joint_velocity_4,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_5": {
            "position": joint_position_5,
            "effort": joint_effort_5,
            "velocity": joint_velocity_5,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_6": {
            "position": joint_position_6,
            "effort": joint_effort_6,
            "velocity": joint_velocity_6,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_7": {
            "position": joint_position_7,
            "effort": joint_effort_7,
            "velocity": joint_velocity_7,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        },
        "joint_8": {
            "position": joint_position_8,
            "effort": joint_effort_8,
            "velocity": joint_velocity_8,
            "imu": {
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular_velocity": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "linear_acceleration": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
        }
    }
    # 1. Read file contents
    with open(json_file_1, "r") as file:
        dataFromJsonFile = json.load(file)

    # 2. Update json object
    dataFromJsonFile["frames"].append(entry)

    # 3. Write json file
    with open(json_file_1, "w") as file:
        json.dump(dataFromJsonFile, file)

    if str(data.header.seq) == "320":
        jointStates_sub.unregister()
        print("")
        print("")
        rospy.signal_shutdown("end")
        sys.exit(0)


def read_json_data(json_data):
    # Opening JSON file
    with open(json_data) as file:
        json_data = json.load(file)

    return json_data


def get_effort_sum(json_data, num_samples):
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "joint_8"]
    effort = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    for i in range(len(listOfJoints)):
        effort_ = 0
        for j in range(num_samples):
            effort_ = effort_ + abs(json_data["frames"][j][listOfJoints[i]]["effort"])
        effort[i] = effort_
    return effort


def get_angular_acceleration(json_data, num_samples):
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    angular_acceleration_array = np.empty(shape=(len(listOfJoints), num_samples - 1))
    angular_acceleration_array.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, num_samples):
            pre_stamp = json_data["frames"][j - 1]["header"]["stamp"][0:10] + "." + \
                        json_data["frames"][j - 1]["header"]["stamp"][10:]
            act_stamp = json_data["frames"][j]["header"]["stamp"][0:10] + "." + json_data["frames"][j]["header"][
                                                                                    "stamp"][10:]
            delta_time = float(act_stamp) - float(pre_stamp)
            acc = json_data["frames"][j][listOfJoints[i]]["velocity"] - json_data["frames"][j - 1][listOfJoints[i]][
                "velocity"]

            angular_acceleration_array[i][j - 1] = acc / delta_time

    return angular_acceleration_array


def plot_effort(json_data_0, json_data_1, num_samples, joint_num):
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    effort_array_0 = np.empty(shape=(len(listOfJoints), num_samples - 1))
    effort_array_0.fill(0)

    effort_array_1 = np.empty(shape=(len(listOfJoints), num_samples - 1))
    effort_array_1.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, num_samples):
            effort_array_0[i][j - 1] = json_data_0["frames"][j][listOfJoints[i]]["effort"]

    for i in range(len(listOfJoints)):
        for j in range(1, num_samples):
            effort_array_1[i][j - 1] = json_data_1["frames"][j][listOfJoints[i]]["effort"]

    plt.plot(effort_array_0[joint_num])
    plt.plot(effort_array_1[joint_num])

    plt.title(listOfJoints[joint_num])

    plt.xlabel("Number of Epochs")
    plt.ylabel("Torque (Nm)")

    plt.legend(["Initial State", "Optimized State"], loc="lower right")
    plt.show()


def plot_position(json_data_0, json_data_1, num_samples, joint_num):
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    position_array_0 = np.empty(shape=(len(listOfJoints), num_samples - 1))
    position_array_0.fill(0)

    position_array_1 = np.empty(shape=(len(listOfJoints), num_samples - 1))
    position_array_1.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, num_samples):
            position_array_0[i][j - 1] = json_data_0["frames"][j][listOfJoints[i]]["position"]

    for i in range(len(listOfJoints)):
        for j in range(1, num_samples):
            position_array_1[i][j - 1] = json_data_1["frames"][j][listOfJoints[i]]["position"]

    plt.plot(position_array_0[joint_num])
    plt.plot(position_array_1[joint_num])

    plt.title(listOfJoints[joint_num])

    plt.xlabel("Number of Epochs")
    plt.ylabel("Position (Radius)")

    plt.legend(["Initial State", "Optimized State"], loc="lower right")
    plt.show()

    
def get_multiple_angular_acceleration(json_data_0, json_data_1, num_samples, joint_num):
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    angular_acceleration_array_0 = np.empty(shape=(len(listOfJoints), num_samples - 1))
    angular_acceleration_array_0.fill(0)

    angular_acceleration_array_1 = np.empty(shape=(len(listOfJoints), num_samples - 1))
    angular_acceleration_array_1.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, num_samples):
            pre_stamp = json_data_0["frames"][j - 1]["header"]["stamp"][0:10] + "." + \
                        json_data_0["frames"][j - 1]["header"]["stamp"][10:]
            act_stamp = json_data_0["frames"][j]["header"]["stamp"][0:10] + "." + json_data_0["frames"][j]["header"][
                                                                                      "stamp"][10:]
            delta_time = float(act_stamp) - float(pre_stamp)
            acc = json_data_0["frames"][j][listOfJoints[i]]["velocity"] - json_data_0["frames"][j - 1][listOfJoints[i]][
                "velocity"]

            angular_acceleration_array_0[i][j - 1] = acc / delta_time

    for i in range(len(listOfJoints)):
        for j in range(1, num_samples):
            pre_stamp = json_data_1["frames"][j - 1]["header"]["stamp"][0:10] + "." + \
                        json_data_1["frames"][j - 1]["header"]["stamp"][10:]
            act_stamp = json_data_1["frames"][j]["header"]["stamp"][0:10] + "." + json_data_1["frames"][j]["header"][
                                                                                      "stamp"][10:]
            delta_time = float(act_stamp) - float(pre_stamp)
            acc = json_data_1["frames"][j][listOfJoints[i]]["velocity"] - json_data_1["frames"][j - 1][listOfJoints[i]][
                "velocity"]

            angular_acceleration_array_1[i][j - 1] = acc / delta_time

    phase_0 = angular_acceleration_array_0[joint_num]
    phase_1 = angular_acceleration_array_1[joint_num]

    plt.plot(phase_0)
    plt.plot(phase_1)

    plt.title(listOfJoints[joint_num])

    plt.xlabel("Number of Epochs")
    plt.ylabel("Angular Acceleration (º/s²)")

    plt.legend(["Initial State", "Optimized State"], loc="lower right")

    plt.show()


def get_angular_acceleration_sum(ang_acceleration, num_samples):
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    acc = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    for i in range(len(listOfJoints)):
        aux = 0
        for j in range(num_samples - 1):
            aux = aux + abs(ang_acceleration[i][j])
        acc[i] = aux
    return acc


if __name__ == '__main__':
    # Define and parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_data', help='Save data from ROS topics.',
                        required=False, default=False)
    parser.add_argument('--job_name_1', help='Save data from ROS topics.',
                        required=False, default="task1_1")
    parser.add_argument('--job_name_2', help='Save data from ROS topics.',
                        required=False, default="task1_2")
    parser.add_argument('--read_data', help='Read data from json file.',
                        required=False, default=False)
    parser.add_argument('--get_angular_acc_sum', help='Get the sum of angular acceleration of each joint.',
                        required=False, default=False)
    parser.add_argument('--plot_acc', help='Plot the angular acceleration for 3 samples.',
                        required=False, default=False)
    parser.add_argument('--joint_num', help='Name of the joint to plot the angular acceleration for 3 samples.',
                        required=False, default="0")
    parser.add_argument('--plot_effort', help='Plot the effort.',
                        required=False, default=False)
    parser.add_argument('--get_effort_sum', help='Get the sum of torque for each joint.',
                        required=False, default=False)
    parser.add_argument('--plot_position', help='Plot the position of each individual joint',
                        required=False, default=False)
    parser.add_argument('--num_samples', help='Number of samples to plot.',
                        required=False, default="300")

    args = parser.parse_args()

    json_file_1 = "/home/filipe/Desktop/Dissertação/Isaac_Gym/" + args.job_name_1 + ".json"
    json_file_2 = "/home/filipe/Desktop/Dissertação/Isaac_Gym/" + args.job_name_2 + ".json"

    if args.save_data:
        listener_ros_topics()

    if args.read_data:
        jsonData = read_json_data(json_file_1)

    if args.get_angular_acc_sum:
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)
        ang_acceleration_1 = get_angular_acceleration(jsonData_1, int(args.num_samples))
        ang_acceleration_2 = get_angular_acceleration(jsonData_2, int(args.num_samples))
        acc_sum_1 = get_angular_acceleration_sum(ang_acceleration_1, int(args.num_samples))
        acc_sum_2 = get_angular_acceleration_sum(ang_acceleration_2, int(args.num_samples))

        print("Angular acceleration ini: ")
        print(acc_sum_1)
        print("Angular acceleration end: ")
        print(acc_sum_2)

    if args.plot_acc:
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)
        get_multiple_angular_acceleration(jsonData_1, jsonData_2, int(args.num_samples), int(args.joint_num))

    if args.plot_effort:
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)
        plot_effort(jsonData_1, jsonData_2, int(args.num_samples), int(args.joint_num))

    if args.get_effort_sum:
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)
        eff_sum_1 = get_effort_sum(jsonData_1, int(args.num_samples))
        eff_sum_2 = get_effort_sum(jsonData_2, int(args.joint_num))

        print("Torque ini: ")
        print(eff_sum_1)
        print("Torque end: ")
        print(eff_sum_2)

    if args.plot_position:
        jsonData_1 = read_json_data(json_file_1)
        jsonData_2 = read_json_data(json_file_2)
        plot_position(jsonData_1, jsonData_2, int(args.num_samples), int(args.joint_num))
