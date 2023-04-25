#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu
import re
import matplotlib.pyplot as plt
from matplotlib.collections import EventCollection
import message_filters

data_file_joint_position = "/home/filipe/Desktop/Dissertação/data_position.txt"
data_file_joint_velocity = "/home/filipe/Desktop/Dissertação/data_velocity.txt"
data_file_joint_effort = "/home/filipe/Desktop/Dissertação/data_effort.txt"

data_file_imu_link0_orientation = "/home/filipe/Desktop/Dissertação/imu_link0_orientation.txt"
data_file_imu_link0_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link0_angular_velocity.txt"
data_file_imu_link0_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link0_linear_acceleration.txt"

data_file_imu_link1_orientation = "/home/filipe/Desktop/Dissertação/imu_link1_orientation.txt"
data_file_imu_link1_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link1_angular_velocity.txt"
data_file_imu_link1_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link1_linear_acceleration.txt"

data_file_imu_link2_orientation = "/home/filipe/Desktop/Dissertação/imu_link2_orientation.txt"
data_file_imu_link2_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link2_angular_velocity.txt"
data_file_imu_link2_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link2_linear_acceleration.txt"

data_file_imu_link3_orientation = "/home/filipe/Desktop/Dissertação/imu_link3_orientation.txt"
data_file_imu_link3_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link3_angular_velocity.txt"
data_file_imu_link3_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link3_linear_acceleration.txt"

data_file_imu_link4_orientation = "/home/filipe/Desktop/Dissertação/imu_link4_orientation.txt"
data_file_imu_link4_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link4_angular_velocity.txt"
data_file_imu_link4_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link4_linear_acceleration.txt"

data_file_imu_link5_orientation = "/home/filipe/Desktop/Dissertação/imu_link5_orientation.txt"
data_file_imu_link5_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link5_angular_velocity.txt"
data_file_imu_link5_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link5_linear_acceleration.txt"

data_file_imu_link6_orientation = "/home/filipe/Desktop/Dissertação/imu_link6_orientation.txt"
data_file_imu_link6_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link6_angular_velocity.txt"
data_file_imu_link6_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link6_linear_acceleration.txt"

data_file_imu_link7_orientation = "/home/filipe/Desktop/Dissertação/imu_link7_orientation.txt"
data_file_imu_link7_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link7_angular_velocity.txt"
data_file_imu_link7_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link7_linear_acceleration.txt"

data_file_imu_link8_orientation = "/home/filipe/Desktop/Dissertação/imu_link8_orientation.txt"
data_file_imu_link8_angular_velocity = "/home/filipe/Desktop/Dissertação/imu_link8_angular_velocity.txt"
data_file_imu_link8_linear_acceleration = "/home/filipe/Desktop/Dissertação/imu_link8_linear_acceleration.txt"

velocity_read = False
robot_home_position = False


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
        save_joint_states(joints)
        save_imu(imu_link_0, data_file_imu_link0_orientation, data_file_imu_link0_angular_velocity,
                 data_file_imu_link0_linear_acceleration)

        save_imu(imu_link_1, data_file_imu_link1_orientation, data_file_imu_link1_angular_velocity,
                 data_file_imu_link1_linear_acceleration)

        save_imu(imu_link_2, data_file_imu_link2_orientation, data_file_imu_link2_angular_velocity,
                 data_file_imu_link2_linear_acceleration)

        save_imu(imu_link_3, data_file_imu_link3_orientation, data_file_imu_link3_angular_velocity,
                 data_file_imu_link3_linear_acceleration)

        save_imu(imu_link_4, data_file_imu_link4_orientation, data_file_imu_link4_angular_velocity,
                 data_file_imu_link4_linear_acceleration)

        save_imu(imu_link_5, data_file_imu_link5_orientation, data_file_imu_link5_angular_velocity,
                 data_file_imu_link5_linear_acceleration)

        save_imu(imu_link_6, data_file_imu_link6_orientation, data_file_imu_link6_angular_velocity,
                 data_file_imu_link6_linear_acceleration)

        save_imu(imu_link_7, data_file_imu_link7_orientation, data_file_imu_link7_angular_velocity,
                 data_file_imu_link7_linear_acceleration)

        save_imu(imu_link_8, data_file_imu_link8_orientation, data_file_imu_link8_angular_velocity,
                 data_file_imu_link8_linear_acceleration)
    else:
        print("Home Position", flush=True, end="\r")


def save_joint_states(data):
    joint_position = data.position
    joint_velocity = data.velocity
    joint_effort = data.effort
    timeStamp = data.header.stamp

    rospy.loginfo("Timestamp joint           %s", timeStamp)
    #rospy.loginfo("Joint Position %s", joint_position)
    #rospy.loginfo("Joint Velocity %s", joint_velocity)
    #rospy.loginfo("Joint Effort %s", joint_effort)

    file_position = open(data_file_joint_position, "a")
    file_position.write(str(joint_position))
    file_position.close()

    file_velocity = open(data_file_joint_velocity, "a")
    file_velocity.write(str(joint_velocity))
    file_velocity.close()

    file_effort = open(data_file_joint_effort, "a")
    file_effort.write(str(joint_effort))
    file_effort.close()


def save_imu(data, orientation_file_name, angular_velocity_file_name, linear_acceleration_file_name):
    orientation = data.orientation
    angular_velocity = data.angular_velocity
    linear_acceleration = data.linear_acceleration
    timeStamp = data.header.stamp

    rospy.loginfo("Timestamp IMU             %s", timeStamp)
    #rospy.loginfo("Orientation         %s", orientation)
    #rospy.loginfo("Angular Velocity    %s", angular_velocity)
    #rospy.loginfo("Linear Acceleration %s", linear_acceleration)

    file_position = open(orientation_file_name, "a")
    file_position.write(str(orientation))
    file_position.close()

    file_velocity = open(angular_velocity_file_name, "a")
    file_velocity.write(str(angular_velocity))
    file_velocity.close()

    file_effort = open(linear_acceleration_file_name, "a")
    file_effort.write(str(linear_acceleration))
    file_effort.close()


def listener_ros_topics():
    file_position = open(data_file_joint_position, "w")
    file_position.close()

    file_velocity = open(data_file_joint_velocity, "w")
    file_velocity.close()

    file_effort = open(data_file_joint_effort, "w")
    file_effort.close()

    file = open(data_file_imu_link0_orientation, "w")
    file.close()
    file = open(data_file_imu_link0_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link0_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link1_orientation, "w")
    file.close()
    file = open(data_file_imu_link1_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link1_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link2_orientation, "w")
    file.close()
    file = open(data_file_imu_link2_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link2_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link3_orientation, "w")
    file.close()
    file = open(data_file_imu_link3_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link3_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link4_orientation, "w")
    file.close()
    file = open(data_file_imu_link4_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link4_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link5_orientation, "w")
    file.close()
    file = open(data_file_imu_link5_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link5_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link6_orientation, "w")
    file.close()
    file = open(data_file_imu_link6_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link6_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link7_orientation, "w")
    file.close()
    file = open(data_file_imu_link7_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link7_linear_acceleration, "w")
    file.close()

    file = open(data_file_imu_link8_orientation, "w")
    file.close()
    file = open(data_file_imu_link8_angular_velocity, "w")
    file.close()
    file = open(data_file_imu_link8_linear_acceleration, "w")
    file.close()
    print("Start listening ")
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
        x_result[i] = x_result[i].replace('[', '').replace("'", "").replace(']', '')\
            .replace(',', '')
        x.append(x_result[i])

    for j in range(0, len(y_result)):
        y_result[j] = y_result[j].replace('[', '').replace("'", "").replace(']', '')\
            .replace(',', '')
        y.append(y_result[j])

    for k in range(0, len(z_result)):
        z_result[k] = z_result[k].replace('[', '').replace("'", "").replace(']', '')\
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


if __name__ == '__main__':
    listener_ros_topics()

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
