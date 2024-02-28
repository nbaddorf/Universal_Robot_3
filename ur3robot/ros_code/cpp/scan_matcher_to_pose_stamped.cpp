#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>


ros::Publisher pose_pub;

void pos_sub_callback(const geometry_msgs::Pose2D &msg) {

  if (pose_pub.getNumSubscribers() > 0) {
    geometry_msgs::PoseWithCovarianceStamped poseStamped;
    tf2::Quaternion theta_quat;

    poseStamped.header.frame_id = "odom";
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose.pose.position.x = msg.x;
    poseStamped.pose.pose.position.y = msg.y;
    poseStamped.pose.pose.position.z = 0.0;

    theta_quat.setRPY(0,0,msg.theta);
    theta_quat = theta_quat.normalize();

    poseStamped.pose.pose.orientation.x = theta_quat[0];
    poseStamped.pose.pose.orientation.y = theta_quat[1];
    poseStamped.pose.pose.orientation.z = theta_quat[2];
    poseStamped.pose.pose.orientation.w = theta_quat[3];
    auto covArray(36, 0.01);
    poseStamped.pose.covariance = covArray;
    
    pose_pub.publish(poseStamped);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher_to_pose_stamped");

  ros::NodeHandle n;

  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("laser_scan_macher/pose_stamped", 10);
  ros::Subscriber pos_sub = n.subscribe("laser_scan_macher/pose2D", 10, pos_sub_callback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}