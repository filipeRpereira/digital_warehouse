#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import re
import matplotlib.pyplot as plt
from matplotlib.collections import EventCollection



data_file_joint_position = "/home/filipe/Desktop/Dissertação/data_position.txt"
data_file_joint_velocity = "/home/filipe/Desktop/Dissertação/data_velocity.txt"
data_file_joint_effort   = "/home/filipe/Desktop/Dissertação/data_effort.txt"

velocity_read = False 

def callback(data):
    joint_position = data.position
    joint_velocity = data.velocity
    joint_effort = data.effort
    # '0.0', ' -0.7853', ' -0.0001', ' -1.5715', ' 0.0', ' 1.0423', ' 0.0', ' 0.0347', ' 0.0353)']

    home_position = str(joint_position).split(',')
    
    for i in range(len(home_position)):
        home_position[i] = home_position[i].replace('(','').replace(')','')
    
    home_values = [0.0, -0.7853, 0.0001, -1.5715, 0.0, 1.0423, 0.0, 0.0347, 0.0353]
    dif_robot = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    dif = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    for i in range(len(home_position)):
        dif_robot[i] = float(home_position[i])


    for i in range(len(dif_robot)):
        dif[i] = abs(dif_robot[i] - home_values[i])
    
    home = True
    for i in range(len(dif)):
        if dif[i] >0.01:
            home = False

    if home == False:
        print("Home Position")

        rospy.loginfo("Joint Position %s", joint_position)
        rospy.loginfo("Joint Velocity %s", joint_velocity)
        rospy.loginfo("Joint Effort %s", joint_effort)
        
        file_position = open(data_file_joint_position, "a")
        file_position.write(str(joint_position))
        file_position.close()

        file_velocity = open(data_file_joint_velocity, "a")
        file_velocity.write(str(joint_velocity))
        file_velocity.close()

        file_effort = open(data_file_joint_effort, "a")
        file_effort.write(str(joint_effort))
        file_effort.close()
    
    else:
        print("Not Home Position - saving data")


def listener_new():
    file_position = open(data_file_joint_position, "w")
    file_position.close()
    
    file_velocity = open(data_file_joint_velocity, "w")
    file_velocity.close()
    
    file_effort = open(data_file_joint_effort, "w")
    file_effort.close()

    rospy.init_node('listener_new', anonymous=False)
    rospy.Subscriber("joint_states_issac", JointState, callback)
    rospy.spin()


def read_joints_data_from_file(data_file):
    joints = []

    text_file = open(data_file, "r")
    filetext = text_file.read()
    text_file.close()

    result = str(re.findall('(.*)',filetext))
    result = result.split(')')

    for i in range(0,len(result)):
        result[i] = result[i].replace('(','')
    
    for i in range(1,len(result)-1):
        joints.append(result[i].split(','))
        #print(result[i])
        
    return joints
    

def plot_results(joints_data, y_label):
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

if __name__ == '__main__':
    #listener_new()
    joints_position = read_joints_data_from_file(data_file_joint_position)
    joints_velocity = read_joints_data_from_file(data_file_joint_velocity)
    joints_effort = read_joints_data_from_file(data_file_joint_effort)

    #plot_results(joints_position, "Position")
    plot_results(joints_velocity, "Velocity")
    #plot_results(joints_effort, "Effort")

    
