#!/usr/bin/python3

import numpy as np
import rospy
from wam_msgs.msg import RTJointPos, RTJointVel
from wam_srvs.srv import JointMove

import pickle
import os


joint_state_data = []

POS_READY = [
    0.002227924477643431, 
    -0.1490540623980915, 
    -0.04214558734519736, 
    1.6803055108189549, 
    0.06452207850075688, 
    -0.06341508205589094, 
    0.01366506663019359,
]

def go_ready_pos():
        """Move WAM to a desired ready position.
        """
        joint_move(POS_READY)

def joint_move(pos_goal: np.ndarray):
        """Move WAM to a desired position.
        q is a numpy array of length 7 that specifies the joint angles
        """
        # Communicate with /wam/joint_move service on control computer
        rospy.wait_for_service('/wam/joint_move')
        try:
            #print('found service')
            joint_move_service = rospy.ServiceProxy('/wam/joint_move', JointMove)
            joint_move_service(pos_goal)
            #print('called move_q')
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

def joint_vel_cmd(vel_goal: np.ndarray,
                  jnt_vel_pub):
        msg = RTJointVel()
        # Publish to ROS
        msg.velocities = vel_goal
        #print(rospy.get_time())
        jnt_vel_pub.publish(msg)

def clip_velocity(vel, max_norm):
        vel_norm = np.linalg.norm(vel)
        if vel_norm > max_norm:
            print("clipped vel")
            return vel/vel_norm*max_norm # velocity rescaled to have max norm
        else:
            return vel


def joint_pos_cmd(pos_goal: np.ndarray, jnt_pos_pub):
                      
        msg = RTJointPos()
        # Publish to ROS
        msg.joints = pos_goal
        msg.rate_limits = np.array([500.0]*7)
        jnt_pos_pub.publish(msg)


def clip_joint_offset(offset):
    if offset > np.pi/3:
        offset = np.pi/3
    
    elif offset < -1*np.pi/3:
        offset = -1*np.pi/3
    
    return offset

'''
Sends velocity commands ~200 Hz
Reads and stores pos, vel, torque (effort) at ~400 Hz
'''

if __name__ == '__main__':

    rospy.init_node("trajecory_node")
    # Create a ROS publisher
    jnt_vel_pub = rospy.Publisher('/wam/jnt_vel_cmd', RTJointVel, queue_size=1)
    jnt_pos_pub = rospy.Publisher('/wam/jnt_pos_cmd', RTJointPos, queue_size=1)
    # Create a rate
    rate = rospy.Rate(200)
    i = 0
    go_ready_pos()

    os.chdir("/home/robot/trjtemp")

    trj = []
    with open("trj.pickle", 'rb') as file:
        trj = pickle.load(file)

    # print(trj)

    for point in trj:
        print(point)
        joint_move(list(point))
        if rospy.is_shutdown():
            break

    # for i in range(n):
    #     print(i)
    #     pos = [1.2,3,4,5,7]
    #     joint_move(pos)

    