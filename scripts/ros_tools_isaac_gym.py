#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu
import re
import matplotlib.pyplot as plt
from matplotlib.collections import EventCollection
import message_filters
import json
import numpy as np
import time
import argparse

#job_name = "job_2"

time_test = 20000
time_started = 0

robot_home_position = False


## OK
def callback_check_home_position(joints):
    joint_position = joints.position
    home_position = str(joint_position).split(',')

    for i in range(len(home_position)):
        home_position[i] = home_position[i].replace('(', '').replace(')', '')

    home_values = [0.012, -0.5697, 0.0, -2.8105, 0.0, 3.0312, 0.741, 0.04, 0.04]
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

    global robot_home_position

    if not home:
        print("Not Home Position", flush=True, end="\r")
        save_json_data(joints)

    else:
        print("Home Position", flush=True, end="\r")


## OK
def listener_ros_topics():
    json_object = json.dumps({
        "frames": []
    }, indent=2)

    with open(json_file, "w") as outfile:
        outfile.write(json_object)

    rospy.init_node('listener_new', anonymous=False)
    jointStates_sub = message_filters.Subscriber('joint_states', JointState)

    ts = message_filters.TimeSynchronizer([jointStates_sub], 10)
    ts.registerCallback(callback_check_home_position)
    rospy.spin()


## OK
def save_json_data(data):
    joint_velocity_0 = data.velocity[0]
    joint_effort_0 = data.effort[0]
    if str(joint_velocity_0) == "nan":
        joint_velocity_0 = 0
    if str(joint_effort_0) == "nan":
        joint_effort_0 = 0

    joint_velocity_1 = data.velocity[1]
    joint_effort_1 = data.effort[1]
    if str(joint_velocity_1) == "nan":
        joint_velocity_1 = 0
    if str(joint_effort_1) == "nan":
        joint_effort_1 = 0

    joint_velocity_2 = data.velocity[2]
    joint_effort_2 = data.effort[2]
    if str(joint_velocity_2) == "nan":
        joint_velocity_2 = 0
    if str(joint_effort_2) == "nan":
        joint_effort_2 = 0

    joint_velocity_3 = data.velocity[3]
    joint_effort_3 = data.effort[3]
    if str(joint_velocity_3) == "nan":
        joint_velocity_3 = 0
    if str(joint_effort_3) == "nan":
        joint_effort_3 = 0

    joint_velocity_4 = data.velocity[4]
    joint_effort_4 = data.effort[4]
    if str(joint_velocity_4) == "nan":
        joint_velocity_4 = 0
    if str(joint_effort_4) == "nan":
        joint_effort_4 = 0

    joint_velocity_5 = data.velocity[5]
    joint_effort_5 = data.effort[5]
    if str(joint_velocity_5) == "nan":
        joint_velocity_5 = 0
    if str(joint_effort_5) == "nan":
        joint_effort_5 = 0

    joint_velocity_6 = data.velocity[6]
    joint_effort_6 = data.effort[6]
    if str(joint_velocity_6) == "nan":
        joint_velocity_6 = 0
    if str(joint_effort_6) == "nan":
        joint_effort_6 = 0

    joint_velocity_7 = data.velocity[7]
    joint_effort_7 = data.effort[7]
    if str(joint_velocity_7) == "nan":
        joint_velocity_7 = 0
    if str(joint_effort_7) == "nan":
        joint_effort_7 = 0

    joint_velocity_8 = data.velocity[8]
    joint_effort_8 = data.effort[8]
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
    with open(json_file, "r") as file:
        dataFromJsonFile = json.load(file)

    # 2. Update json object
    dataFromJsonFile["frames"].append(entry)

    # 3. Write json file
    with open(json_file, "w") as file:
        json.dump(dataFromJsonFile, file)


def read_json_data(json_file):
    # Opening JSON file
    with open(json_file) as file:
        jsonData = json.load(file)

    return jsonData


## Colocar unidades nos gráficos
def plot_data(json_data):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    for joint in listOfJoints:
        print(joint)

        '''++++++++++++++++   linear_acceleration   ++++++++++++++++'''
        acc_x = []
        acc_y = []
        acc_z = []
        for i in range(len(jsonData["frames"])):
            acc_x.append(jsonData["frames"][i][joint]["imu"]["linear_acceleration"]["x"])
        for i in range(len(jsonData["frames"])):
            acc_y.append(jsonData["frames"][i][joint]["imu"]["linear_acceleration"]["y"])
        for i in range(len(jsonData["frames"])):
            acc_z.append(jsonData["frames"][i][joint]["imu"]["linear_acceleration"]["z"])

        fig1, axs1 = plt.subplots(3)
        axs1[0].plot(acc_x)
        axs1[0].set_title('Linear Acceleration X - ' + joint)
        axs1[1].plot(acc_y)
        axs1[1].set_title('Linear Acceleration Y - ' + joint)
        axs1[2].plot(acc_z)
        axs1[2].set_title('Linear Acceleration Z - ' + joint)
        fig1.tight_layout()
        yLabel = "Linear Acceleration"
        for ax in axs1.flat:
            ax.set(xlabel='samples', ylabel=yLabel)
        for ax in axs1.flat:
            ax.label_outer()
        # plt.show()

        '''++++++++++++++++   angular_velocity   ++++++++++++++++'''
        vel_x = []
        vel_y = []
        vel_z = []
        for i in range(len(jsonData["frames"])):
            vel_x.append(jsonData["frames"][i][joint]["imu"]["angular_velocity"]["x"])
        for i in range(len(jsonData["frames"])):
            vel_y.append(jsonData["frames"][i][joint]["imu"]["angular_velocity"]["y"])
        for i in range(len(jsonData["frames"])):
            vel_z.append(jsonData["frames"][i][joint]["imu"]["angular_velocity"]["z"])

        fig2, axs1 = plt.subplots(3)
        axs1[0].plot(vel_x)
        axs1[0].set_title('Angular Velocity X - ' + joint)
        axs1[1].plot(vel_y)
        axs1[1].set_title('Angular Velocity Y - ' + joint)
        axs1[2].plot(vel_z)
        axs1[2].set_title('Angular Velocity Z - ' + joint)
        fig2.tight_layout()
        yLabel = "Angular Velocity"
        for ax in axs1.flat:
            ax.set(xlabel='samples', ylabel=yLabel)
        for ax in axs1.flat:
            ax.label_outer()
        # plt.show()

        '''++++++++++++++++   orientation   ++++++++++++++++'''
        ori_x = []
        ori_y = []
        ori_z = []
        for i in range(len(jsonData["frames"])):
            ori_x.append(jsonData["frames"][i][joint]["imu"]["orientation"]["x"])
        for i in range(len(jsonData["frames"])):
            ori_y.append(jsonData["frames"][i][joint]["imu"]["orientation"]["y"])
        for i in range(len(jsonData["frames"])):
            ori_z.append(jsonData["frames"][i][joint]["imu"]["orientation"]["z"])

        fig3, axs1 = plt.subplots(3)
        axs1[0].plot(ori_x)
        axs1[0].set_title('Orientation X - ' + joint)
        axs1[1].plot(ori_y)
        axs1[1].set_title('Orientation Y - ' + joint)
        axs1[2].plot(ori_z)
        axs1[2].set_title('Orientation Z - ' + joint)
        fig3.tight_layout()
        yLabel = "Orientation"
        for ax in axs1.flat:
            ax.set(xlabel='samples', ylabel=yLabel)
        for ax in axs1.flat:
            ax.label_outer()
        # plt.show()

        '''++++++++++++++++   effort   ++++++++++++++++'''
        eff = []
        for i in range(len(jsonData["frames"])):
            eff.append(jsonData["frames"][i][joint]["effort"])

        '''++++++++++++++++   velocity   ++++++++++++++++'''
        vel = []
        for i in range(len(jsonData["frames"])):
            vel.append(jsonData["frames"][i][joint]["velocity"])

        fig4, axs1 = plt.subplots(2)
        axs1[0].plot(eff)
        axs1[0].set_title('Effort - ' + joint)
        axs1[1].plot(vel)
        axs1[1].set_title('Velocity - ' + joint)

        fig4.tight_layout()
        yLabel = ""
        for ax in axs1.flat:
            ax.set(xlabel='samples', ylabel=yLabel)
        for ax in axs1.flat:
            ax.label_outer()
        plt.show()


def get_acc_sum(jsonData):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    acc = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    for i in range(len(listOfJoints)):
        acc_x = 0
        acc_y = 0
        acc_z = 0
        for j in range(len(jsonData["frames"])):
            acc_x = acc_x + abs(jsonData["frames"][j][listOfJoints[i]]["imu"]["linear_acceleration"]["x"])
        for k in range(len(jsonData["frames"])):
            acc_y = acc_y + abs(jsonData["frames"][k][listOfJoints[i]]["imu"]["linear_acceleration"]["y"])
        for m in range(len(jsonData["frames"])):
            acc_z = acc_z + abs(jsonData["frames"][m][listOfJoints[i]]["imu"]["linear_acceleration"]["z"])
        acc[i][0] = acc_x
        acc[i][1] = acc_y
        acc[i][2] = acc_z
    return acc


def get_angular_velocity_sum(jsonData):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    ang_vel = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    for i in range(len(listOfJoints)):
        #print(listOfJoints[i])
        ang_vel_ = 0
        for j in range(len(jsonData["frames"])):
            ang_vel_ = ang_vel_ + abs(jsonData["frames"][j][listOfJoints[i]]["velocity"])
        ang_vel[i] = ang_vel_
    return ang_vel


def get_effort_sum(jsonData):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    effort = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    for i in range(len(listOfJoints)):
        effort_ = 0
        for j in range(len(jsonData["frames"])):
            effort_ = effort_ + abs(jsonData["frames"][j][listOfJoints[i]]["effort"])
        effort[i] = effort_
    return effort


## OK
def get_angular_acceleration(jsonData, plot, num_samples):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    angular_acceleration_array = np.empty(shape=(len(listOfJoints), total_frames-1))
    angular_acceleration_array.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, len(jsonData["frames"])):
            pre_stamp = jsonData["frames"][j-1]["header"]["stamp"][0:10] + "." + jsonData["frames"][j-1]["header"]["stamp"][10:]
            act_stamp = jsonData["frames"][j]["header"]["stamp"][0:10] + "." + jsonData["frames"][j]["header"]["stamp"][10:]
            delta_time = float(act_stamp) - float(pre_stamp)
            acc = jsonData["frames"][j][listOfJoints[i]]["velocity"] - jsonData["frames"][j-1][listOfJoints[i]]["velocity"]

            angular_acceleration_array[i][j-1] = acc/delta_time

    return angular_acceleration_array


def get_effort(jsonData_0, jsonData_1, plot, num_samples, joint_num):
    total_frames_0 = len(jsonData_0["frames"])
    total_frames_1 = len(jsonData_1["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    effort_array_0 = np.empty(shape=(len(listOfJoints), total_frames_0-1))
    effort_array_0.fill(0)

    effort_array_1 = np.empty(shape=(len(listOfJoints), total_frames_1-1))
    effort_array_1.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, len(jsonData_0["frames"])):
            effort_array_0[i][j-1] = jsonData_0["frames"][j][listOfJoints[i]]["effort"]

    for i in range(len(listOfJoints)):
        for j in range(1, len(jsonData_1["frames"])):
            effort_array_1[i][j-1] = jsonData_1["frames"][j][listOfJoints[i]]["effort"]

    plt.plot(effort_array_0[joint_num][0:num_samples])
    plt.plot(effort_array_1[joint_num][0:num_samples])

    plt.title(listOfJoints[joint_num])

    plt.xlabel("Number of Epochs")
    plt.ylabel("Torque (Nm)")

    plt.legend(["Initial State", "Optimized State"], loc="lower right")
    plt.show()


def get_multiple_angular_acceleration(jsonData_0, jsonData_1, plot, num_samples, joint_num):
    total_frames_0 = len(jsonData_0["frames"])
    total_frames_1 = len(jsonData_1["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    angular_acceleration_array_0 = np.empty(shape=(len(listOfJoints), total_frames_0-1))
    angular_acceleration_array_0.fill(0)

    angular_acceleration_array_1 = np.empty(shape=(len(listOfJoints), total_frames_1-1))
    angular_acceleration_array_1.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, len(jsonData_0["frames"])):
            pre_stamp = jsonData_0["frames"][j-1]["header"]["stamp"][0:10] + "." + jsonData_0["frames"][j-1]["header"]["stamp"][10:]
            act_stamp = jsonData_0["frames"][j]["header"]["stamp"][0:10] + "." + jsonData_0["frames"][j]["header"]["stamp"][10:]
            delta_time = float(act_stamp) - float(pre_stamp)
            acc = jsonData_0["frames"][j][listOfJoints[i]]["velocity"] - jsonData_0["frames"][j-1][listOfJoints[i]]["velocity"]

            angular_acceleration_array_0[i][j-1] = acc/delta_time

    for i in range(len(listOfJoints)):
        for j in range(1, len(jsonData_1["frames"])):
            pre_stamp = jsonData_1["frames"][j-1]["header"]["stamp"][0:10] + "." + jsonData_1["frames"][j-1]["header"]["stamp"][10:]
            act_stamp = jsonData_1["frames"][j]["header"]["stamp"][0:10] + "." + jsonData_1["frames"][j]["header"]["stamp"][10:]
            delta_time = float(act_stamp) - float(pre_stamp)
            acc = jsonData_1["frames"][j][listOfJoints[i]]["velocity"] - jsonData_1["frames"][j-1][listOfJoints[i]]["velocity"]

            angular_acceleration_array_1[i][j-1] = acc/delta_time

    fase_0 = angular_acceleration_array_0[joint_num][0:num_samples]
    fase_1 = angular_acceleration_array_1[joint_num][0:num_samples]

    plt.plot(fase_0)
    plt.plot(fase_1)

    plt.title(listOfJoints[joint_num])

    plt.xlabel("Number of Epochs")
    plt.ylabel("Angular Acceleration (º/s²)")

    plt.legend(["Initial State", "Optimized State"], loc="lower right")

    plt.show()


## OK IMPORTANT!!!
def get_angular_acceleration_sum(ang_acceleration, num_samples):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    acc = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    for i in range(len(listOfJoints)):
        aux = 0
        for j in range(num_samples - 1):
            aux = aux + abs(ang_acceleration[i][j])
        acc[i] = aux
    return acc


def histogram(ini_acc, end_acc, joint_num, num_samples):
    fig, ax1 = plt.subplots()
    colors = ['r', 'g']
    ax1.hist([ini_acc[joint_num][:num_samples], end_acc[joint_num][:num_samples]], color=colors,
             label=['Initial State', 'Optimized State'])
    ax1.set_xlim(-30, 30)
    ax1.set_ylabel("Count")
    plt.legend(loc='upper right')

    plt.xlabel("Angular Acceleration (º/s²)")
    plt.ylabel("Number of samples")
    plt.title("Joint " + str(joint_num))

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # Define and parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_data', help='Save data from ROS topics.',
                        required=False, default=False)
    parser.add_argument('--job_name', help='Save data from ROS topics.',
                        required=False, default="Task_1_fase_0")
    parser.add_argument('--job_name_2', help='Save data from ROS topics.',
                        required=False, default="Task_1_fase_2")
    parser.add_argument('--read_data', help='Read data from json file.',
                        required=False, default=False)
    parser.add_argument('--get_angular_acc', help='Get the sum of angular acceleration of each joint.',
                        required=False, default=False)
    parser.add_argument('--plot_acc', help='Plot the angular acceleration.',
                        required=False, default=False)
    parser.add_argument('--plot_all_acc', help='Plot the angular acceleration for 3 samples.',
                        required=False, default=False)
    parser.add_argument('--plot_joint_num', help='Name of the joint to plot the angular acceleration for 3 samples.',
                        required=False, default="0")
    parser.add_argument('--plot_effort', help='Plot the effort.',
                        required=False, default=False)

    args = parser.parse_args()

    job_name = args.job_name
    json_file = "/home/filipe/Desktop/Dissertação/Isaac_Gym/" + args.job_name + ".json"
    json_file_2 = "/home/filipe/Desktop/Dissertação/Isaac_Gym/" + args.job_name_2 + ".json"

    if args.save_data:
        listener_ros_topics()

    if args.read_data:
        jsonData = read_json_data(json_file)


    if args.get_angular_acc:
        jsonData_0 = read_json_data(json_file)
        jsonData_1 = read_json_data(json_file_2)
        ang_acceleration_0 = get_angular_acceleration(jsonData_0, False, 50)
        ang_acceleration_1 = get_angular_acceleration(jsonData_1, False, 50)

        angular_acceleration_sum_0 = get_angular_acceleration_sum(ang_acceleration_0, 50)
        angular_acceleration_sum_1 = get_angular_acceleration_sum(ang_acceleration_1, 50)

        histogram(ang_acceleration_0, ang_acceleration_1, int(args.plot_joint_num), 50)

        print("Angular acceleration ini: ")
        print(angular_acceleration_sum_0)
        print("Angular acceleration end: ")
        print(angular_acceleration_sum_1)

    if args.plot_acc:
        jsonData = read_json_data(json_file)
        get_angular_acceleration(jsonData, args.plot_acc, 50)

    if args.plot_all_acc:
        jsonData_0 = read_json_data(json_file)
        jsonData_1 = read_json_data(json_file_2)
        get_multiple_angular_acceleration(jsonData_0, jsonData_1, args.plot_all_acc, 50,
                                          int(args.plot_joint_num))

    if args.plot_effort:
        jsonData_0 = read_json_data(json_file)
        jsonData_1 = read_json_data(json_file_2)
        get_effort(jsonData_0, jsonData_1, args.plot_effort, 50,
                   int(args.plot_joint_num))