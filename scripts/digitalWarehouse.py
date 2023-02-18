#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
import time

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from moveit_msgs.msg import MoveItErrorCodes
import numpy as np
from geometry_msgs.msg import Quaternion

import ros_tools

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=False)

#pub = rospy.Publisher("/joint_command", JointState, queue_size=20)
joint_command_isaac = JointState()

pub_gripper = rospy.Publisher("/gripper_command", Grasp, queue_size=20)

positions = []
velocities = []
accelerations = []


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        #group_name = "panda_arm"
        #group_name = "panda_hand"
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.group_name = group_name

        joint_goal = self.move_group.get_current_joint_values()
        print("============ Panda Joints:")
        print(joint_goal)
        print("---------------------------")


    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()


        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        ## The go command can be called with joint values, poses, or without any
        ## parameters if you have already set the pose or joint target for the group
        move_group.plan()

        move_group.set_planning_time(10)
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)

        move_group.go(joint_goal, wait=True)

        ## Calling ``stop()`` ensures that there is no residual movement
        #move_group.stop()

        ## END_SUB_TUTORIAL

        ## For testing:
        current_joints = move_group.get_current_joint_values()

        #new_arr_pos = np.concatenate( (current_joints, [0.2, 0.2] ) )
        #joint_command_isaac.position = new_arr_pos    
        #pub.publish(joint_command_isaac)

        print(self.move_group.get_current_pose().pose)



    
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self, x, y, z):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
  
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 1
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0

        # X, Y, Z, W
        #quat_tf = [0, 1, 0, 0]
        #xyz = [x, y, z]
        #print(quat_tf)

        move_group.set_planning_time(5)
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)

        #move_group.set_orientation_target(quat_tf, end_effector_link = "panda_hand")
        move_group.set_pose_target(pose_goal, end_effector_link="panda_hand")
        



        ## Now, we call the planner to compute the plan and execute it.
        ## `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)

        ## Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        ## It is always good to clear your targets after planning with poses.
        ## Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        ## For testing:
        ## Note that since this section of code will not be included in the tutorials
        ## we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose

        #print(self.move_group.get_current_pose().pose)



        return all_close(pose_goal, current_pose, 0.01)


    def go_to_pose_goal_old(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        poses_list = []

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = -1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)


        ## Now, we call the planner to compute the plan and execute it.
        ## `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)

        ## Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        ## It is always good to clear your targets after planning with poses.
        ## Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        ## For testing:
        ## Note that since this section of code will not be included in the tutorials
        ## we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        current_joint_values = self.move_group.get_current_joint_values()
        #print("----------------------------------")
        #print(current_joint_values)
        
        joint_command_isaac.position = np.concatenate( (current_joint_values, [0, 0] ) )    
        pub.publish(joint_command_isaac)

        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.2  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.2  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.2  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x -= scale * 0.2  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.2  # Third move sideways (y)
        wpose.position.z += scale * -0.1  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))
       

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        
        num_joints = len(joint_command_isaac.name)
        #joint_state_position = np.array([0.0] * num_joints)

        for i in range(len(plan.joint_trajectory.points)):
            joint_state_position = np.array(plan.joint_trajectory.points[i].positions)
            new_arr_pos = np.concatenate( (joint_state_position, [0, 0] ) )
            positions.append(new_arr_pos)

            joint_state_velocity = np.array(plan.joint_trajectory.points[i].velocities)
            new_arr_vel = np.concatenate( (joint_state_velocity, [0, 0] ) )
            velocities.append(new_arr_vel)


        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        #move_group.set_max_velocity_scaling_factor(0.5)



        rate = rospy.Rate(20)
        for i in range(len(positions)):
            joint_command_isaac.position = positions[i]
            joint_command_isaac.velocity = velocities[i]
            
            pub.publish(joint_command_isaac)
            rate.sleep()        


        move_group.execute(plan, wait=True)


    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def add_table(self, timeout=4):
            # Copy class variables to local variables to make the web tutorials more clear.
            # In practice, you should use the class variables directly unless you have a good
            # reason not to.
            box_name = self.box_name
            scene = self.scene

            ## BEGIN_SUB_TUTORIAL add_box
            ##
            ## Adding Objects to the Planning Scene
            ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            ## First, we will create a box in the planning scene between the fingers:
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "panda_link0"


            box_pose.pose.position.x = 0.5
            box_pose.pose.position.y = 0
            box_pose.pose.position.z = 0.025


            box_name = "table"
            scene.add_box(box_name, box_pose, size = (0.5, 0.5, 0.15))


            ## END_SUB_TUTORIAL
            # Copy local variables back to class variables. In practice, you should use the class
            # variables directly unless you have a good reason not to.
            self.box_name = box_name
            return self.wait_for_state_update(box_is_known=True, timeout=timeout), box_pose


    def add_table_placement(self, timeout=4):
            # Copy class variables to local variables to make the web tutorials more clear.
            # In practice, you should use the class variables directly unless you have a good
            # reason not to.
            box_name = self.box_name
            scene = self.scene

            ## BEGIN_SUB_TUTORIAL add_box
            ##
            ## Adding Objects to the Planning Scene
            ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            ## First, we will create a box in the planning scene between the fingers:
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "panda_link0"


            box_pose.pose.position.x = -0.5
            box_pose.pose.position.y = 0
            box_pose.pose.position.z = 0.025


            box_name = "table_placement"
            scene.add_box(box_name, box_pose, size = (0.5, 0.5, 0.15))


            ## END_SUB_TUTORIAL
            # Copy local variables back to class variables. In practice, you should use the class
            # variables directly unless you have a good reason not to.
            self.box_name = box_name
            return self.wait_for_state_update(box_is_known=True, timeout=timeout), box_pose


    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0"

        #box_pose.pose.orientation.x = 0.0
        #box_pose.pose.orientation.y = 1.0
        #box_pose.pose.orientation.z = 0.0
        #box_pose.pose.orientation.w = 0.0


        box_pose.pose.position.x = 0.6
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.125


        box_name = "Cube"
        scene.add_box(box_name, box_pose, size = (0.05, 0.05, 0.05))


        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout), box_pose


    def pick_box(self, box):   
        rospy.sleep(1)
        rospy.logwarn("moving to test")
        grasps = [] 
        #0.67611; 0.0091003; 0.71731
        g = Grasp()
        g.id = "test"
        
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "panda_link0"
        grasp_pose.pose.position.x = box.pose.position.x
        grasp_pose.pose.position.y = box.pose.position.y
        grasp_pose.pose.position.z = box.pose.position.z + 0.1

        grasp_pose.pose.orientation.x = 0
        grasp_pose.pose.orientation.y = 1
        grasp_pose.pose.orientation.z = 0
        grasp_pose.pose.orientation.w = 0


        rospy.logwarn("moving to arm")
        move_group = self.move_group
        
        rospy.sleep(1)
        
        # set the grasp pose
        g.grasp_pose = grasp_pose

        # define the pre-grasp approach
        g.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        g.pre_grasp_approach.direction.vector.x = box.pose.position.x #- 2.0
        g.pre_grasp_approach.direction.vector.y = box.pose.position.y + 0.4
        g.pre_grasp_approach.direction.vector.z = -box.pose.position.z - 0.2
        g.pre_grasp_approach.min_distance = 0.1
        g.pre_grasp_approach.desired_distance = 0.3
        g.pre_grasp_posture.header.frame_id = "panda_link0"
        g.pre_grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
    
        pos = JointTrajectoryPoint()
        pos.positions.append(0.06)
    
        g.pre_grasp_posture.points.append(pos)
    
        # set the grasp posture
        g.grasp_posture.header.frame_id = "panda_link0"
        g.grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

        pos = JointTrajectoryPoint()
        pos.positions.append(0.0)
        pos.effort.append(0.9)
    
        g.grasp_posture.points.append(pos)
    
        # set the post-grasp retreat
        g.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        g.post_grasp_retreat.direction.vector.x = -0.4
        g.post_grasp_retreat.direction.vector.y = 0.2
        g.post_grasp_retreat.direction.vector.z = 0.7
        g.post_grasp_retreat.desired_distance = 0.25
        g.post_grasp_retreat.min_distance = 0.01

        #g.allowed_touch_objects = ["table", "panda_hand", "Cube"]
        g.allowed_touch_objects = ["table"]
        
        g.max_contact_force = 0.0
        g.grasp_quality = 0.1     
        
        # append the grasp to the list of grasps
        grasps.append(g)
        
        # pick the object
        #move_group.pick("Cube", grasps)
        result = False
        n_attempts = 0
           
        ## repeat until will succeed
        while result == False:
            print("Attempts pickup: "), n_attempts
            result = move_group.pick("Cube", grasps)      
            n_attempts += 1
            rospy.sleep(1)
        rospy.loginfo("Pickup successful")

        placement_pose = PoseStamped()
        placement_pose.header.frame_id = "panda_link0"
        placement_pose.pose.position.x = -0.5
        placement_pose.pose.position.y = 0.0
        placement_pose.pose.position.z = 0.2

        placement_pose.pose.orientation.x = 0.0
        placement_pose.pose.orientation.y = 0
        placement_pose.pose.orientation.z = 1.
        placement_pose.pose.orientation.w = 0

        result1 = False
        n_attempts1 = 0
        while result1 == False:
            print("Attempts place: "), n_attempts1
            result1 = move_group.place("Cube", placement_pose)      
            n_attempts1 += 1
            rospy.sleep(0.2)
        rospy.sleep(1)
        rospy.loginfo("Placement successful")
    

    def pick_box_inverted(self, box):   
        rospy.sleep(1)
        rospy.logwarn("moving to test")
        grasps = [] 
        g = Grasp()
        g.id = "test"
        
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "panda_link0"

        grasp_pose.pose.position.x = -0.4
        grasp_pose.pose.position.y = 0.0
        grasp_pose.pose.position.z = 0.2
        
        grasp_pose.pose.orientation.y = 1.0
        grasp_pose.pose.orientation.z = 0
        grasp_pose.pose.orientation.w = 0


        rospy.logwarn("moving to arm")
        move_group = self.move_group
        
        rospy.sleep(1)
        
        # set the grasp pose
        g.grasp_pose = grasp_pose

        # define the pre-grasp approach
        g.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        #g.pre_grasp_approach.direction.vector.x = box.pose.position.x #- 2.0
        #g.pre_grasp_approach.direction.vector.y = box.pose.position.y + 0.4
        #g.pre_grasp_approach.direction.vector.z = -box.pose.position.z - 0.2
        g.pre_grasp_approach.direction.vector.x = -0.5
        g.pre_grasp_approach.direction.vector.y = 0.4
        g.pre_grasp_approach.direction.vector.z = 0.235
        g.pre_grasp_approach.min_distance = 0.1
        g.pre_grasp_approach.desired_distance = 0.3
        g.pre_grasp_posture.header.frame_id = "panda_link0"
        g.pre_grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

        pos = JointTrajectoryPoint()
        pos.positions.append(0.06)

        g.pre_grasp_posture.points.append(pos)

        # set the grasp posture
        g.grasp_posture.header.frame_id = "panda_link0"
        g.grasp_posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

        pos = JointTrajectoryPoint()
        pos.positions.append(0.0)
        pos.effort.append(0.9)

        g.grasp_posture.points.append(pos)

        # set the post-grasp retreat
        g.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        g.post_grasp_retreat.direction.vector.x = -0.4
        g.post_grasp_retreat.direction.vector.y = 0.2
        g.post_grasp_retreat.direction.vector.z = 0.7
        g.post_grasp_retreat.desired_distance = 0.25
        g.post_grasp_retreat.min_distance = 0.01

        #g.allowed_touch_objects = ["table", "panda_hand", "Cube"]
        g.allowed_touch_objects = ["table"]
        
        g.max_contact_force = 0.0
        g.grasp_quality = 0.1     
        
        # append the grasp to the list of grasps
        grasps.append(g)
        
        # pick the object
        #move_group.pick("Cube", grasps)
        result = False
        n_attempts = 0
            
        ## repeat until will succeed
        while result == False:
            print("Attempts pickup: "), n_attempts
            result = move_group.pick("Cube", grasps)      
            n_attempts += 1
            rospy.sleep(1)
        rospy.loginfo("Pickup successful")

        placement_pose = PoseStamped()
        placement_pose.header.frame_id = "panda_link0"
        placement_pose.pose.position.x = -0.5
        placement_pose.pose.position.y = 0.0
        placement_pose.pose.position.z = 0.125

        placement_pose.pose.orientation.x = 0.0
        placement_pose.pose.orientation.y = 0
        placement_pose.pose.orientation.z = 1.0
        placement_pose.pose.orientation.w = 0

        result1 = False
        n_attempts1 = 0
        while result1 == False:
            print("Attempts place: "), n_attempts1
            result1 = move_group.place("Cube", placement_pose)      
            n_attempts1 += 1
            rospy.sleep(0.2)
        rospy.sleep(1)
        rospy.loginfo("Placement successful")
    

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )


    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )


    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )



def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the Digital Warehouse project")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

       
        
        #input(
        #    "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        #)
        tutorial = MoveGroupPythonInterfaceTutorial()
        tutorial.add_table()
        tutorial.add_table_placement()

        input("Press any key to continue...")

        start = time.time()
        tutorial.go_to_joint_state()
                                    # x, y, z
        tutorial.go_to_pose_goal(0.3, -0.2, 0.32)
        tutorial.go_to_pose_goal(0.3, -0.2, 0.22)
        tutorial.go_to_pose_goal(0.3, -0.2, 0.32)

        tutorial.go_to_pose_goal(0.7, -0.2, 0.32)
        tutorial.go_to_pose_goal(0.7, -0.2, 0.22)
        tutorial.go_to_pose_goal(0.7, -0.2, 0.32)

        tutorial.go_to_pose_goal(0.7, 0.2, 0.32)
        tutorial.go_to_pose_goal(0.7, 0.2, 0.22)
        tutorial.go_to_pose_goal(0.7, 0.2, 0.32)

        tutorial.go_to_pose_goal(0.3, 0.2, 0.32)
        tutorial.go_to_pose_goal(0.3, 0.2, 0.22)
        tutorial.go_to_pose_goal(0.3, 0.2, 0.32)


        tutorial.go_to_pose_goal(-0.3, 0.2, 0.32)
        tutorial.go_to_pose_goal(-0.3, 0.2, 0.22)
        tutorial.go_to_pose_goal(-0.3, 0.2, 0.32)

        tutorial.go_to_pose_goal(-0.7, 0.2, 0.32)
        tutorial.go_to_pose_goal(-0.7, 0.2, 0.22)
        tutorial.go_to_pose_goal(-0.7, 0.2, 0.32)

        tutorial.go_to_pose_goal(-0.7, -0.2, 0.32)
        tutorial.go_to_pose_goal(-0.7, -0.2, 0.22)
        tutorial.go_to_pose_goal(-0.7, -0.2, 0.32)

        tutorial.go_to_pose_goal(-0.3, -0.2, 0.32)
        tutorial.go_to_pose_goal(-0.3, -0.2, 0.22)
        tutorial.go_to_pose_goal(-0.3, -0.2, 0.32)


        tutorial.go_to_joint_state()

        print("time: ")
        end = time.time()
        print(end - start)
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
