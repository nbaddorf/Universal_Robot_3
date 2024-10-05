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
        val = float(raw_input("choose rotation 1 value: "))
        point.x = val
        newVal = float(raw_input("axis3: "))
        point.y = newVal
        newestVal = float(raw_input("axis z: "))
        point.z = newestVal
        pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
