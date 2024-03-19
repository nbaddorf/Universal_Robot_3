#!/usr/bin/env python

import rospkg
import yaml
import rospy
import sys
import argparse
import os
import tf
from geometry_msgs.msg import PoseStamped

##################################################################
# to use just run python save_robot_waypoints.py NAMEOFWAYPOINT
# where NAMEOFWAYPOINT is whatever you want. Then go into Rviz and click a place
#on the map with the 2d nav goal pointer. This program then saves the location
#into a yaml file in the python_scripts/yaml_files folder
#################################################################

rospack = rospkg.RosPack()
packageDir = rospack.get_path('ur3master')

def print_waypoint_names():
        global packageDir
        with open(packageDir + "/yaml_files/waypoint_positions.yaml") as file:
            documents = yaml.full_load(file)
            print("")
            print("current waypoint names are: ")
            for i in documents:
                print(i)

parser = argparse.ArgumentParser(description='Name of waypoint')
parser.add_argument('name', metavar='n', type=str,
                    help='name of waypoint')

try:
    args = parser.parse_args()
    posName = args.name
except:
    print_waypoint_names()
    exit()

def appendToYaml(trans, rot):
    global posName
    dataRot = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
    data_to_write = {posName : dataRot}
    with open(packageDir + "/yaml_files/waypoint_positions.yaml", 'a') as file:
        documents = yaml.dump(data_to_write, file)
        #sys.exit()
    rospy.signal_shutdown("finnished saving to yaml")

def goalCB(msg):
    global posName
    rospy.loginfo("Received at goal message!")
    rospy.loginfo("Timestamp: " + str(msg.header.stamp))
    rospy.loginfo("frame_id: " + str(msg.header.frame_id))
    rospy.loginfo("saving waypoint name as: " + str(posName))

    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
    rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

    pos = [position.x, position.y, position.z]
    rot = [quat.x, quat.y, quat.z, quat.w]

    appendToYaml(pos,rot) 

if __name__ == "__main__":
    rospy.init_node('robot_waypoint_saver_tool', anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCB)
    rospy.spin()