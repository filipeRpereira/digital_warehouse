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

json_file = "/home/filipe/Desktop/Dissertação/json_file.json"

robot_home_position = False


## OK
def callback_check_home_position(joints, imu_link_0, imu_link_1, imu_link_2, imu_link_3, imu_link_4, imu_link_5,
                                 imu_link_6, imu_link_7, imu_link_8):
    joint_position = joints.position
    # '0.0', ' -0.7853', ' -0.0001', ' -1.5715', ' 0.0', ' 1.0423', ' 0.0', ' 0.0347', ' 0.0353)']

    home_position = str(joint_position).split(',')

    for i in range(len(home_position)):
        home_position[i] = home_position[i].replace('(', '').replace(')', '')

    home_values = [0.0, -0.7853, 0.0001, -1.5715, 0.0, 1.0423, 0.0, 0.0347, 0.0353]
    dif_robot = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    dif = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    for i in range(len(home_position)):
        dif_robot[i] = float(home_position[i])

    for i in range(len(dif_robot)):
        dif[i] = abs(dif_robot[i] - home_values[i])

    home = True
    for i in range(len(dif)):
        if dif[i] > 0.01:
            home = False

    global robot_home_position

    if not home:
        print("Not Home Position", flush=True, end="\r")
        save_json_data(joints, imu_link_0, imu_link_1, imu_link_2, imu_link_3,
                       imu_link_4, imu_link_5, imu_link_6, imu_link_7, imu_link_8)

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
    jointStates_sub = message_filters.Subscriber('joint_states_issac', JointState)
    imuLink0_sub = message_filters.Subscriber('imu_link0', Imu)
    imuLink1_sub = message_filters.Subscriber('imu_link1', Imu)
    imuLink2_sub = message_filters.Subscriber('imu_link2', Imu)
    imuLink3_sub = message_filters.Subscriber('imu_link3', Imu)
    imuLink4_sub = message_filters.Subscriber('imu_link4', Imu)
    imuLink5_sub = message_filters.Subscriber('imu_link5', Imu)
    imuLink6_sub = message_filters.Subscriber('imu_link6', Imu)
    imuLink7_sub = message_filters.Subscriber('imu_link7', Imu)
    imuLink8_sub = message_filters.Subscriber('imu_link8', Imu)

    ts = message_filters.TimeSynchronizer([jointStates_sub, imuLink0_sub, imuLink1_sub, imuLink2_sub, imuLink3_sub,
                                           imuLink4_sub, imuLink5_sub, imuLink6_sub, imuLink7_sub, imuLink8_sub], 10)
    ts.registerCallback(callback_check_home_position)
    rospy.spin()


## OK
def save_json_data(data, imu_link_0, imu_link_1, imu_link_2, imu_link_3, imu_link_4, imu_link_5,
                   imu_link_6, imu_link_7, imu_link_8):
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
                    "x": imu_link_0.orientation.x,
                    "y": imu_link_0.orientation.y,
                    "z": imu_link_0.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_0.angular_velocity.x,
                    "y": imu_link_0.angular_velocity.y,
                    "z": imu_link_0.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_0.linear_acceleration.x,
                    "y": imu_link_0.linear_acceleration.y,
                    "z": imu_link_0.linear_acceleration.z
                }
            }
        },
        "joint_1": {
            "effort": joint_effort_1,
            "velocity": joint_velocity_1,
            "imu": {
                "orientation": {
                    "x": imu_link_1.orientation.x,
                    "y": imu_link_1.orientation.y,
                    "z": imu_link_1.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_1.angular_velocity.x,
                    "y": imu_link_1.angular_velocity.y,
                    "z": imu_link_1.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_1.linear_acceleration.x,
                    "y": imu_link_1.linear_acceleration.y,
                    "z": imu_link_1.linear_acceleration.z
                }
            }
        },
        "joint_2": {
            "effort": joint_effort_2,
            "velocity": joint_velocity_2,
            "imu": {
                "orientation": {
                    "x": imu_link_2.orientation.x,
                    "y": imu_link_2.orientation.y,
                    "z": imu_link_2.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_2.angular_velocity.x,
                    "y": imu_link_2.angular_velocity.y,
                    "z": imu_link_2.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_2.linear_acceleration.x,
                    "y": imu_link_2.linear_acceleration.y,
                    "z": imu_link_2.linear_acceleration.z
                }
            }
        },
        "joint_3": {
            "effort": joint_effort_3,
            "velocity": joint_velocity_3,
            "imu": {
                "orientation": {
                    "x": imu_link_3.orientation.x,
                    "y": imu_link_3.orientation.y,
                    "z": imu_link_3.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_3.angular_velocity.x,
                    "y": imu_link_3.angular_velocity.y,
                    "z": imu_link_3.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_3.linear_acceleration.x,
                    "y": imu_link_3.linear_acceleration.y,
                    "z": imu_link_3.linear_acceleration.z
                }
            }
        },
        "joint_4": {
            "effort": joint_effort_4,
            "velocity": joint_velocity_4,
            "imu": {
                "orientation": {
                    "x": imu_link_4.orientation.x,
                    "y": imu_link_4.orientation.y,
                    "z": imu_link_4.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_4.angular_velocity.x,
                    "y": imu_link_4.angular_velocity.y,
                    "z": imu_link_4.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_4.linear_acceleration.x,
                    "y": imu_link_4.linear_acceleration.y,
                    "z": imu_link_4.linear_acceleration.z
                }
            }
        },
        "joint_5": {
            "effort": joint_effort_5,
            "velocity": joint_velocity_5,
            "imu": {
                "orientation": {
                    "x": imu_link_5.orientation.x,
                    "y": imu_link_5.orientation.y,
                    "z": imu_link_5.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_5.angular_velocity.x,
                    "y": imu_link_5.angular_velocity.y,
                    "z": imu_link_5.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_5.linear_acceleration.x,
                    "y": imu_link_5.linear_acceleration.y,
                    "z": imu_link_5.linear_acceleration.z
                }
            }
        },
        "joint_6": {
            "effort": joint_effort_6,
            "velocity": joint_velocity_6,
            "imu": {
                "orientation": {
                    "x": imu_link_6.orientation.x,
                    "y": imu_link_6.orientation.y,
                    "z": imu_link_6.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_6.angular_velocity.x,
                    "y": imu_link_6.angular_velocity.y,
                    "z": imu_link_6.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_6.linear_acceleration.x,
                    "y": imu_link_6.linear_acceleration.y,
                    "z": imu_link_6.linear_acceleration.z
                }
            }
        },
        "joint_7": {
            "effort": joint_effort_7,
            "velocity": joint_velocity_7,
            "imu": {
                "orientation": {
                    "x": imu_link_7.orientation.x,
                    "y": imu_link_7.orientation.y,
                    "z": imu_link_7.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_7.angular_velocity.x,
                    "y": imu_link_7.angular_velocity.y,
                    "z": imu_link_7.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_7.linear_acceleration.x,
                    "y": imu_link_7.linear_acceleration.y,
                    "z": imu_link_7.linear_acceleration.z
                }
            }
        },
        "joint_8": {
            "effort": joint_effort_8,
            "velocity": joint_velocity_8,
            "imu": {
                "orientation": {
                    "x": imu_link_8.orientation.x,
                    "y": imu_link_8.orientation.y,
                    "z": imu_link_8.orientation.z
                },
                "angular_velocity": {
                    "x": imu_link_8.angular_velocity.x,
                    "y": imu_link_8.angular_velocity.y,
                    "z": imu_link_8.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": imu_link_8.linear_acceleration.x,
                    "y": imu_link_8.linear_acceleration.y,
                    "z": imu_link_8.linear_acceleration.z
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


## OK
def read_json_data():
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
        #print(listOfJoints[i])
        '''++++++++++++++++   linear_acceleration   ++++++++++++++++'''
        acc_x = 0
        acc_y = 0
        acc_z = 0
        for j in range(len(jsonData["frames"])):
            acc_x = acc_x + abs(jsonData["frames"][j][listOfJoints[i]]["imu"]["linear_acceleration"]["x"])
        for k in range(len(jsonData["frames"])):
            acc_y = acc_y + abs(jsonData["frames"][k][listOfJoints[i]]["imu"]["linear_acceleration"]["y"])
        for m in range(len(jsonData["frames"])):
            acc_z = acc_z + abs(jsonData["frames"][m][listOfJoints[i]]["imu"]["linear_acceleration"]["z"])

        #print("total_frames: ", total_frames)
        #print("acc_x: ", acc_x)
        #print("acc_y: ", acc_y)
        #print("acc_z: ", acc_z)
        #print()
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

        #print("angular velocity: ", ang_vel_)
        #print()
        ang_vel[i] = ang_vel_
    #print("total_frames: ", total_frames)
    return ang_vel


def get_effort_sum(jsonData):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]

    effort = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    for i in range(len(listOfJoints)):
        #print(listOfJoints[i])
        effort_ = 0
        for j in range(len(jsonData["frames"])):
            effort_ = effort_ + abs(jsonData["frames"][j][listOfJoints[i]]["effort"])

        #print("effort: ", effort_)
        #print()
        effort[i] = effort_
    #print("total_frames: ", total_frames)
    return effort


def get_execution_time(jsonData):
    total_frames = len(jsonData["frames"])
    sampling_time = 42 / total_frames
    #print("sampling_time (tf): ", sampling_time)
    #print("Execution Time: ", sampling_time * total_frames)
    execution_time = sampling_time * total_frames
    return execution_time, sampling_time


def get_angular_acceleration(jsonData):
    total_frames = len(jsonData["frames"])
    listOfJoints = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "joint_7", "joint_8"]
    angular_acceleration_array = np.empty(shape=(len(listOfJoints), total_frames-1))
    angular_acceleration_array.fill(0)

    for i in range(len(listOfJoints)):
        for j in range(1, len(jsonData["frames"])):
            angular_acceleration_array[i][j-1] = jsonData["frames"][j][listOfJoints[i]]["velocity"] - jsonData["frames"][j-1][listOfJoints[i]]["velocity"]

    fig1, axs1 = plt.subplots(3)
    axs1[0].plot(angular_acceleration_array[0])
    axs1[0].set_title('Aceleração Angular - Junta 0')
    axs1[1].plot(angular_acceleration_array[1])
    axs1[1].set_title('Aceleração Angular - Junta 1')
    axs1[2].plot(angular_acceleration_array[2])
    axs1[2].set_title('Aceleração Angular - Junta 2')

    fig1.tight_layout()
    yLabel = "º/s²"
    for ax in axs1.flat:
        ax.set(xlabel='Amostras', ylabel=yLabel)

    fig2, axs1 = plt.subplots(3)
    axs1[0].plot(angular_acceleration_array[3])
    axs1[0].set_title('Aceleração Angular - Junta 3')
    axs1[1].plot(angular_acceleration_array[4])
    axs1[1].set_title('Aceleração Angular - Junta 4')
    axs1[2].plot(angular_acceleration_array[5])
    axs1[2].set_title('Aceleração Angular - Junta 5')

    fig2.tight_layout()
    yLabel = "º/s²"
    for ax in axs1.flat:
        ax.set(xlabel='Amostras', ylabel=yLabel)

    fig3, axs1 = plt.subplots(3)
    axs1[0].plot(angular_acceleration_array[6])
    axs1[0].set_title('Aceleração Angular - Junta 6')
    axs1[1].plot(angular_acceleration_array[7])
    axs1[1].set_title('Aceleração Angular - Junta 7')
    axs1[2].plot(angular_acceleration_array[8])
    axs1[2].set_title('Aceleração Angular - Junta 8')

    fig3.tight_layout()
    yLabel = "º/s²"
    for ax in axs1.flat:
        ax.set(xlabel='Amostras', ylabel=yLabel)

    #plt.show()

    return angular_acceleration_array


if __name__ == '__main__':
    #listener_ros_topics()
    jsonData = read_json_data()

    #execution_time, sampling_time = get_execution_time(jsonData)
    #acc = get_acc_sum(jsonData)
    #ang_vel = get_angular_velocity_sum(jsonData)
    #eff = get_effort_sum(jsonData)
    ang_acceleration = get_angular_acceleration(jsonData)




    # joints_position = read_joints_data_from_file(data_file_joint_position)
    # joints_velocity = read_joints_data_from_file(data_file_joint_velocity)
    # joints_effort = read_joints_data_from_file(data_file_joint_effort)

    # imu_x_ang_vel, imu_y_ang_vel, imu_z_ang_vel = read_imu_data_from_file(data_file_imu_link5_angular_velocity)
    # imu_x_lin_acc, imu_y_lin_acc, imu_z_lin_acc = read_imu_data_from_file(data_file_imu_link5_linear_acceleration)
    # imu_x_ori, imu_y_ori, imu_z_ori = read_imu_data_from_file(data_file_imu_link5_orientation)

    # plot_imu_results(imu_x_ang_vel, imu_y_ang_vel, imu_z_ang_vel, "Angular Velocity")
    # plot_imu_results(imu_x_lin_acc, imu_y_lin_acc, imu_z_lin_acc, "Linear Acceleration")
    # plot_imu_results(imu_x_ori, imu_y_ori, imu_z_ori, "Orientation")

    # plot_joints_results(joints_velocity, "Velocity")
    # plot_results(joints_effort, "Effort")

    # save_json_data()
