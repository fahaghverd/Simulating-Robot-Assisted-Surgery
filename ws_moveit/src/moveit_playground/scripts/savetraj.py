#!/usr/bin/python3

import rospy
import pickle
import os

from moveit_msgs.msg import RobotTrajectory

def callback(data : RobotTrajectory):
    trj = []
    for point in data.joint_trajectory.points:
        trj.append(point.positions)

    with open("trj.pickle", 'wb') as file:
        pickle.dump(trj, file)
    

if __name__ == "__main__":
    rospy.init_node("savetrj")

    os.chdir("/home/hulk/trjtemp")

    rospy.Subscriber("trjbuffer", RobotTrajectory, callback)

    rospy.spin()