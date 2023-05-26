#!/usr/bin/python3

# import rospy
import pickle
import os

if __name__ == "__main__":
    # rospy.init_node("trajecory_node")
    # Create a ROS publisher
    # jnt_vel_pub = rospy.Publisher('/wam/jnt_vel_cmd', RTJointVel, queue_size=1)
    # jnt_pos_pub = rospy.Publisher('/wam/jnt_pos_cmd', RTJointPos, queue_size=1)
    # Create a rate
    # rate = rospy.Rate(200)
    # i = 0
    # go_ready_pos()

    os.chdir("/home/hulk/trjtemp")

    trj = []
    with open("trj.pickle", 'rb') as file:
        trj = pickle.load(file)


    for point in trj:
        print(list(point))