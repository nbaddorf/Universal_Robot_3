#!/usr/bin/env python

# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('tilt_angle_nick', Float64, queue_size=10)
    rospy.init_node('tilt_anglenick', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(10)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass