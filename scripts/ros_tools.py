#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import re

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
    #print(home_position)
    
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
        print(result[i])
        
    return joints
    

if __name__ == '__main__':
    listener_new()
    #joints_velocity = read_joints_data_from_file(data_file_joint_velocity)
