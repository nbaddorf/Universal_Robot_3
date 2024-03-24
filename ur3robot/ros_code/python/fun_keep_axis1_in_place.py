#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('fun_head_turner')

    listener = tf.TransformListener()

    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    #spawner(4, 2, 0, 'turtle2')

    head_pos = rospy.Publisher('arm_pos', Point, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        point = Point()
        (roll, pitch, yaw) = euler_from_quaternion (rot)
        point.x = yaw
        print(yaw)
        head_pos.publish(point)

        rate.sleep()