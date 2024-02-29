#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <vector>

using std::vector;

ros::Publisher vel_pub;

struct odomStruct {
  vector<double> x;
  vector<double> y;
  vector<double> theta;
};

odomStruct odometry_vector;


void odometry_sub_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  if (odometry_vector.x.size() < 20) {
    odometry_vector.x.push_back(msg->pose.pose.position.x);
    odometry_vector.y.push_back(msg->pose.pose.position.y);
  } else {
    std::cout << "x values: " << std::endl;
    for (auto xiter : odometry_vector.x) {
      std::cout << xiter << std::endl;
    }
    std::cout << "y values: " << std::endl;
    for (auto yiter : odometry_vector.y) {
      std::cout << yiter << std::endl;
    }
    ros::shutdown();
  }
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_analyzer");

  ros::NodeHandle n;
  signal(SIGINT, mySigintHandler);

  //pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("laser_scan_macher/pose_stamped", 10);
  ros::Subscriber odometry_sub = n.subscribe("odometry/filtered", 10, odometry_sub_callback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}