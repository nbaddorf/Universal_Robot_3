#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>


ros::Publisher pose_pub;

struct myPose {
  double x;
  double y;
  double th;
}


void pos_sub_callback(const geometry_msgs::Pose2D &msg) {
  //myPose recieved_pos;
  //recieved_pos.x = msg.x;
  //recieved_pos.y = msg.y;
  //recieved_pos.th = msg.th;

  geometry_msgs::PoseWithCovarianceStamped poseStamped;

  poseStamped.header = msg->header;
  poseStamped.pose.pose.position.x = msg->x;
  poseStamped.pose.pose.position.y = msg->y;
  poseStamped.pose.pose.position.z = 0.0;

  pose_pub.publish(poseStamped);
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher_to_pose_stamped");

  ros::NodeHandle n;

  pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("laser_scan_macher/pose", 10);
  ros::Subscriber pos_sub = n.subscribe("laser_scan_macher/pose2D", 10, pos_sub_callback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}