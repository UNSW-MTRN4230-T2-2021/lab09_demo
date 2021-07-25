#!/usr/bin/python
#

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

import os

import rospy
import tf
import math
import time 

from std_srvs.srv import Empty 
from gazebo_msgs.srv import SpawnModel

from geometry_msgs.msg import *


# Moves ur5e to sepcific join position (do not modify)
def move_ur5e_to(pub, pos, duration_time, wait_time):

    start = time.time()
    while time.time() - start < wait_time:

        traj = JointTrajectory()
        traj.header = Header()
        # Joint names for UR5
        traj.joint_names = ['elbow_joint','shoulder_lift_joint','shoulder_pan_joint', 
                            'wrist_1_joint', 'wrist_2_joint',
                            'wrist_3_joint']

        rate = rospy.Rate(10)

        traj.header.stamp = rospy.Time.now()
        points = JointTrajectoryPoint()
        points.positions = pos
        points.time_from_start = rospy.Duration(duration_time)
        
        # Set the points to the trajectory
        traj.points = []
        traj.points.append(points)

        # Publish the message   
        pub.publish(traj)
        rospy.sleep(0.5)

def spawn_box(pose):

    #INSERT CODE HERE

    pass
    
    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    fp = open('src/ur5e_gazebo/models/cube/cube.sdf')
    
    sdf = fp.read()
    fp.close()

    spawn_model('spawnedCubePython',sdf,'/',pose,"world")


def main():

    rospy.init_node('lab08_python')

    print("generating box (students to do)")
    ot = tf.transformations.quaternion_from_euler(0,0,0)
    pose = Pose(position = Point(x=1.71, y=0.1348, z=0.775), orientation = Quaternion(x=ot[0], y=ot[1], z=ot[2], w=ot[3]))
    spawn_box(pose)
    time.sleep(1)


    joint_pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)

    print("moving to above box")
    time.sleep(1)
  
    pos = [(math.pi/9.1),-(math.pi/9.1),0.0,-(math.pi/2), 3*(math.pi/2), 0.0]
    move_ur5e_to(joint_pub, pos, 1.0, 3)
    
    print("moving down to pick up box")
    time.sleep(1)

    pos = [(math.pi/11.1),-(math.pi/11.1),0.0,-(math.pi/2), 3*(math.pi/2), 0.0]
    move_ur5e_to(joint_pub, pos, 1.0, 3)

    print("turning on gripper (students to do)")

    #INSERT CODE HERE TO TURN GRIPPER ON

    rospy.wait_for_service('/ur5e/epick/on')
    turn_on_gripper = rospy.ServiceProxy('/ur5e/epick/on',Empty)

    turn_on_gripper()
    time.sleep(1)
    print("moving box")
    time.sleep(1)

    pos = [(math.pi/4),-(math.pi/4),0.0,-(math.pi/2), 3*(math.pi/2), 0.0]
    move_ur5e_to(joint_pub, pos, 8.0, 15)

   
    print("moving box back down")
    time.sleep(1)
    rospy.wait_for_service('/ur5e/epick/on')

    pos = [(math.pi/10.5),-(math.pi/10.5),0.0,-(math.pi/2), 3*(math.pi/2), 0.0]
    move_ur5e_to(joint_pub, pos, 8.0, 16)

    print("turning off gripper (students to do)")

    #INSERT CODE HERE TO TURN GRIPPER OFF

    rospy.wait_for_service('/ur5e/epick/off')
    turn_off_gripper = rospy.ServiceProxy('/ur5e/epick/off',Empty)
    turn_off_gripper()

    time.sleep(1)
    print("moving robot away from block")
    time.sleep(1)

    pos = [(math.pi/3),-(math.pi/3),0.0,-(math.pi/2), 3*(math.pi/2), 0.0]
    move_ur5e_to(joint_pub, pos, 2.0, 6)

       


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
