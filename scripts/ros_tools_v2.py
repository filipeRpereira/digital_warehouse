#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu
import re
import matplotlib.pyplot as plt
from matplotlib.collections import EventCollection
import message_filters
import json

json_file = "/home/filipe/Desktop/Dissertação/json_file.json"

velocity_read = False
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


def read_joints_data_from_file(data_file):
    joints = []

    text_file = open(data_file, "r")
    filetext = text_file.read()
    text_file.close()

    result = str(re.findall('(.*)', filetext))
    result = result.split(')')

    for i in range(0, len(result)):
        result[i] = result[i].replace('(', '')

    for i in range(1, len(result) - 1):
        joints.append(result[i].split(','))

    for k in range(0, len(joints)):
        for z in range(0, 9):
            if joints[k][z] == " nan":
                joints[k][z] = '0.0'
            if joints[k][z] == "nan":
                joints[k][z] = '0.0'

    return joints


def read_imu_data_from_file(data_file):
    x = []
    y = []
    z = []

    text_file = open(data_file, "r")
    filetext = text_file.read()
    text_file.close()

    x_result = str(re.findall('x: (.*)', filetext))
    x_result = x_result.split(' ')

    y_result = str(re.findall('y: (.*)', filetext))
    y_result = y_result.split(' ')

    z_result = str(re.findall('z: (.*)', filetext))
    z_result = z_result.split(' ')

    for i in range(0, len(x_result)):
        x_result[i] = x_result[i].replace('[', '').replace("'", "").replace(']', '') \
            .replace(',', '')
        x.append(x_result[i])

    for j in range(0, len(y_result)):
        y_result[j] = y_result[j].replace('[', '').replace("'", "").replace(']', '') \
            .replace(',', '')
        y.append(y_result[j])

    for k in range(0, len(z_result)):
        z_result[k] = z_result[k].replace('[', '').replace("'", "").replace(']', '') \
            .replace(',', '').replace('x:', '')
        z.append(z_result[k])

    ## CONVERT DATA TO FLOAT
    x_data = []
    for data in x:
        x_data.append(float(data))

    y_data = []
    for data in y:
        y_data.append(float(data))

    z_data = []
    for data in z:
        z_data.append(float(data))

    return x_data, y_data, z_data


def plot_joints_results(joints_data, y_label):
    joint_0 = []
    joint_1 = []
    joint_2 = []
    joint_3 = []
    joint_4 = []
    joint_5 = []
    joint_6 = []
    joint_7 = []
    joint_8 = []

    for data in joints_data:
        joint_0.append(float(data[0]))
        joint_1.append(float(data[1]))
        joint_2.append(float(data[2]))
        joint_3.append(float(data[3]))
        joint_4.append(float(data[4]))
        joint_5.append(float(data[5]))
        joint_6.append(float(data[6]))
        joint_7.append(float(data[7]))
        joint_8.append(float(data[8]))

    fig1, axs1 = plt.subplots(3)

    axs1[0].plot(joint_0)
    axs1[0].set_title('joint_0')
    axs1[1].plot(joint_1)
    axs1[1].set_title('joint_1')
    axs1[2].plot(joint_2)
    axs1[2].set_title('joint_2')

    fig2, axs2 = plt.subplots(3)
    axs2[0].plot(joint_3)
    axs2[0].set_title('joint_3')
    axs2[1].plot(joint_4)
    axs2[1].set_title('joint_4')
    axs2[2].plot(joint_5)
    axs2[2].set_title('joint_5')

    fig3, axs3 = plt.subplots(3)
    axs3[0].plot(joint_6)
    axs3[0].set_title('joint_6')
    axs3[1].plot(joint_7)
    axs3[1].set_title('joint_7')
    axs3[2].plot(joint_8)
    axs3[2].set_title('joint_8')

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()

    for ax in axs1.flat:
        ax.set(xlabel='samples', ylabel=y_label)
    for ax in axs2.flat:
        ax.set(xlabel='samples', ylabel=y_label)
    for ax in axs3.flat:
        ax.set(xlabel='samples', ylabel=y_label)

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for ax in axs1.flat:
        ax.label_outer()
    for ax in axs2.flat:
        ax.label_outer()
    for ax in axs3.flat:
        ax.label_outer()
    plt.show()


def plot_imu_results(x, y, z, y_label):
    fig1, axs1 = plt.subplots(3)

    axs1[0].plot(x)
    axs1[0].set_title('X')
    axs1[1].plot(y)
    axs1[1].set_title('Y')
    axs1[2].plot(z)
    axs1[2].set_title('Z')

    fig1.tight_layout()

    for ax in axs1.flat:
        ax.set(xlabel='samples', ylabel=y_label)

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for ax in axs1.flat:
        ax.label_outer()

    plt.show()


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


def read_json_data():
    # Opening JSON file
    with open(json_file) as file:
        jsonData = json.load(file)

    return jsonData


if __name__ == '__main__':
    #listener_ros_topics()
    jsonData = read_json_data()

    # Print the type of data variable
    print("Type:", type(jsonData))

    # Print the data of dictionary
    print(jsonData["frames"][0]["joint_0"]["imu"]["linear_acceleration"])
    print(len(jsonData["frames"]))

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
