#!/usr/bin/env python

import rospy
import sys
from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse 
import tf2_ros
import tf
from geometry_msgs.msg import Quaternion
import rospkg
import geometry_msgs.msg
import time
from tf import TransformListener
import math


def pub_marker_tf(msg, markID):
    br = tf2_ros.TransformBroadcaster()
    #br = tf.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    #qt_msg = Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera_rgb_optical_frame"
    t.child_frame_id = str(markID)
    t.transform.translation.x = msg.position.x #* -1
    t.transform.translation.y = msg.position.y #* -1
    t.transform.translation.z = msg.position.z
    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z #* -1
    t.transform.rotation.w = msg.orientation.w #* -1
    #t.transform.rotation = qt_msg
    #print(qt_msg)

    br.sendTransform(t)

def pub_marker(msg, markID):
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    #marker.id = 0
    marker.ns = str(markID)
    marker.type = marker.CUBE
    marker.action = marker.ADD

    marker.pose.orientation.x = msg.orientation.x
    marker.pose.orientation.y = msg.orientation.y
    marker.pose.orientation.z = msg.orientation.z #* -1
    marker.pose.orientation.w = msg.orientation.w #* -1

    marker.pose.position.x = msg.position.x #* -1
    marker.pose.position.y = msg.position.y #* -1
    marker.pose.position.z = msg.position.z

    t = rospy.Duration()
    marker.lifetime = t
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.02
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.b = 1.0
    marker.color.g = 1.0
    marker_pub.publish(marker)

def markerCB(msg):
    newMsg=msg.markers[0]
    runCurrent = True
    if runCurrent:
        pub_marker(newMsg.pose.pose, newMsg.id)
        #pub_marker_tf(newMsg.pose.pose, newMsg.id)
        #getTransformAndWrite(newMsg.id)
        #print(newMsg.id)

    else:
        print("that tag is excluded")

if __name__ == '__main__':

    rospy.init_node("record_aruco_on_map", anonymous=True)
    marker_sub = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, markerCB)
    marker_pub = rospy.Publisher("/map_recorder/aruco_points", Marker, queue_size = 10)
    tf_listener = tf.TransformListener()

    rospy.spin()
    sys.exit()
