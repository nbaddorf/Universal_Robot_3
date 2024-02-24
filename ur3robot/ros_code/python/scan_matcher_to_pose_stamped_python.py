#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse

def poseCb(data):
    #print(data.x)
    pose_pub(data.x, data.y, data.theta)
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
def pose_pub(x,y,theta):
    pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    pos.header.frame_id = "odom" #base_link
    pos.header.stamp = rospy.Time.now()
    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    pos.pose.pose.position.x = x
    pos.pose.pose.position.y = y
    pos.pose.pose.position.z = 0.0

    quat = quaternion_from_euler (0, 0, theta)

    pos.pose.pose.orientation.x = quat[0]
    pos.pose.pose.orientation.y = quat[1]
    pos.pose.pose.orientation.z = quat[2]
    pos.pose.pose.orientation.w = quat[3]

    pos.pose.covariance[0] = 0 #1.0533996225e-07 #was 0.2
    pos.pose.covariance[7] = 0 #7.5730596059e-08 #was 0.2
    pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    pos.pose.covariance[35] = 0 #0.05 #1.61800759713e-06 #was 0.2
    pub.publish(pos)


if __name__ == '__main__':
    try:
        rospy.init_node('scan_macher_to_pose_stamped', anonymous=True)
        pub = rospy.Publisher('laser_scan_macher/pose_stamped', PoseWithCovarianceStamped, queue_size=5)
        rospy.Subscriber("laser_scan_macher/pose2D", Pose2D, poseCb)
        
        #rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass