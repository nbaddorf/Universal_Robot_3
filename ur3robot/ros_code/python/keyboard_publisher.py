#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('scara/arm_pos', Point, queue_size=10)
    rospy.init_node('keyboard_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        point = Point()
        val = float(raw_input("choose rotation value: "))
        point.x = val
        point.y = 0
        point.z = 0
        pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass