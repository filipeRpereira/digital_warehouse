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

time_test = 20000
time_started = 0

rospy.init_node('listener_new', anonymous=False)
jointStates_sub = message_filters.Subscriber('joint_states_gym', JointState)


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
        save_json_data(joints)

    else:
        print("Saving ROS data...", flush=True, end="\r")


def listener_ros_topics():
    json_object = json.dumps({
        "frames": []
    }, indent=2)

    with open(json_file, "w") as outfile:
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
    with open(json_file, "r") as file:
        dataFromJsonFile = json.load(file)

    # 2. Update json object
    dataFromJsonFile["frames"].append(entry)

    # 3. Write json file
    with open(json_file, "w") as file:
        json.dump(dataFromJsonFile, file)

    if str(data.header.seq) == "320":
        jointStates_sub.unregister()
        print("")
        print("")
        rospy.signal_shutdown("end")
        sys.exit(0)


# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def read_json_data():
    # Opening JSON file
    with open(json_file) as file:
        jsonData = json.load(file)

    return jsonData


def get_valid_frames():
    valid_seq = []

    for i in range(1, len(jsonData["frames"]) - 1):
        previous_stamp = datetime.fromtimestamp(int(jsonData["frames"][i-1]["header"]["stamp"])/1e9)
        previous_stamp = previous_stamp.strftime('%H:%M:%S.%f')

        actual_stamp = datetime.fromtimestamp(int(jsonData["frames"][i]["header"]["stamp"])/1e9)
        actual_stamp = actual_stamp.strftime('%H:%M:%S.%f')

        if actual_stamp[9] != previous_stamp[9]:
            valid_seq.append(jsonData["frames"][i]["header"]["seq"])
            # for debug
            # print("---------------------------------------------------------")
            # print("Sequence  : ", jsonData["frames"][i]["header"]["seq"])
            # print("Timestamp : ", jsonData["frames"][i]["header"]["stamp"])
            # print("Time      : ", actual_stamp)

    return valid_seq


def total_effort_per_joint(valid_seq):
    total_frames = len(jsonData["frames"])
    effort = np.empty(shape=(len(listOfJoints)))
    effort.fill(0)

    for i in range(len(listOfJoints)):
        # For debug only
        #print("-------------------- Joint " + str(i) + " --------------------")

        for j in range(0, total_frames):
            if jsonData["frames"][j]["header"]["seq"] in valid_seq:
                effort[i] += abs(jsonData["frames"][j][listOfJoints[i]]["effort"])
                # For debug only
                #print("Seq    : ", str(jsonData["frames"][j]["header"]["seq"]))
                #print("Effort :      " + str(jsonData["frames"][j][listOfJoints[i]]["effort"]))

    return effort


def get_cycle_time():
    init_stamp = datetime.fromtimestamp(int(jsonData["frames"][0]["header"]["stamp"]) / 1e9)
    end_stamp = datetime.fromtimestamp(int(jsonData["frames"][len(jsonData["frames"])-1]["header"]["stamp"]) / 1e9)

    start_time = str(init_stamp.time())
    end_time = str(end_stamp.time())

    # convert time string to datetime
    t1 = datetime.strptime(start_time, "%H:%M:%S.%f")
    print('Start time:', t1.time())
    t2 = datetime.strptime(end_time, "%H:%M:%S.%f")
    print('End time:', t2.time())
    delta = t2 - t1

    return delta.total_seconds()


def get_effort_array(valid_seq):
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


def get_angular_acc_array(valid_frames):
    total_frames = len(jsonData["frames"])
    data_array = np.empty(shape=(len(listOfJoints), len(valid_frames)))
    data_array.fill(0)

    for i in range(len(listOfJoints)):
        # For debug only
        #print("-------------------- Joint " + str(i) + " --------------------")

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

                # for debug only
                #print("seq in jsonData             : ", jsonData["frames"][j]["header"]["seq"])
                #print("idx_valid_frames            : ", idx_valid_frames)
                #print("next_idx_value_valid_frames : ", next_idx_value_valid_frames)
                #print("valid_frames_next_frame     : ", valid_frames_next_frame)
                #print("velocity_now                : ", velocity_now)
                #print("velocity_next               : ", velocity_next)
                #print("delta velocity              : ", (velocity_next - velocity_now))
                #print("")

    return data_array


def get_valid_timestamp_frame(valid_frames):
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


def plot_effort_sim(_effort_array, _timestamp_array, joint_num):
    plt.title("Reinforcement Learning - " + listOfJoints[joint_num])
    plt.xlabel("Duration (s)")
    plt.ylabel("Torque (Nm)")

    plt.plot(_timestamp_array, _effort_array[joint_num])
    plt.xticks(np.linspace(0.0, timestamp_array[-1], num=10))
    plt.xticks(rotation=90)

    plt.show()


def plot_angular_acc_sim(_angular_acc_array, _timestamp_array, joint_num):
    plt.title("Reinforcement Learning - " + listOfJoints[joint_num])
    plt.xlabel("Duration (s)")
    plt.ylabel("Angular Acceleration (rad/s²)")

    plt.plot(_timestamp_array, _angular_acc_array[joint_num])
    plt.xticks(rotation=90)

    plt.show()


def get_position_array(valid_frames):
    total_frames = len(jsonData["frames"])
    data_array = np.empty(shape=(len(listOfJoints), len(valid_frames)))
    data_array.fill(0)

    for i in range(len(listOfJoints)):
        aux = 0
        for j in range(0, total_frames):
            if jsonData["frames"][j]["header"]["seq"] in valid_frames:
                data_array[i][aux] = jsonData["frames"][j][listOfJoints[i]]["position"]
                aux += 1

    return data_array


def plot_position_sim(_position_array, _timestamp_array, _joint_num):
    plt.title("Reinforcement Learning - " + listOfJoints[_joint_num])
    plt.xlabel("Duration (s)")
    plt.ylabel("Position")

    plt.plot(_timestamp_array, _position_array[_joint_num])
    plt.xticks(np.linspace(0.0, timestamp_array[-1], num=10))
    plt.xticks(rotation=90)

    plt.show()


if __name__ == '__main__':
    # Define and parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_data', help='Save data from ROS topics.',
                        required=False, default=False)
    parser.add_argument('--task_name', help='Save data from ROS topics.',
                        required=False, default="phase_2_0")
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

    json_file = "/home/filipe/Desktop/Dissertação/" + args.task_name + ".json"

    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    if args.save_data:
        listener_ros_topics()

    if args.read_data:
        jsonData = read_json_data()
        valid_frames = get_valid_frames()
        effort_joint = total_effort_per_joint(valid_frames)
        cycle_time = get_cycle_time()
        print("total effort : " + str(round(sum(effort_joint))) + " Nm")
        print("Cycle time   : " + str(round(float(cycle_time), 2)) + " s")

    if args.plot_acc:
        jsonData = read_json_data()
        valid_frames = get_valid_frames()
        angular_acc_array = get_angular_acc_array(valid_frames)
        timestamp_array = get_valid_timestamp_frame(valid_frames)
        plot_angular_acc_sim(angular_acc_array, timestamp_array, int(args.joint_num))

    if args.plot_effort:
        jsonData = read_json_data()
        valid_frames = get_valid_frames()
        effort_array = get_effort_array(valid_frames)
        timestamp_array = get_valid_timestamp_frame(valid_frames)
        plot_effort_sim(effort_array, timestamp_array, int(args.joint_num))

    if args.plot_position:
        jsonData = read_json_data()
        valid_frames = get_valid_frames()
        position_array = get_position_array(valid_frames)
        timestamp_array = get_valid_timestamp_frame(valid_frames)
        plot_position_sim(position_array, timestamp_array, int(args.joint_num))














