#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>

using std::vector;

vector<double> position;
bool start_recording = false;

void jointState_callback(const sensor_msgs::JointState &msg) {
  
  if (start_recording) { 
    position.push_back(msg.position[0]); 
  }
  
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_analyzer");

  ros::NodeHandle n;

  //pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("laser_scan_macher/pose_stamped", 10);
  ros::Subscriber jointState_sub = n.subscribe("ur3/scara/joint_states", 10, jointState_callback);

  ros::Rate loop_rate(10);

  int size_counter = 0;

  while (ros::ok()) {
    if (position.size() < loopNumber || odom_vector.x.size() < loopNumber) {
      ros::spinOnce();
      loop_rate.sleep();
    } else {

      
      vector<double> *vec;

      if (loopType == 0) {
        vec = &odometry_vector.x;
       // std::cout << "odometry x" << std::endl;
      } else if (loopType == 1) {
        vec = &odometry_vector.y;
        //std::cout << "odometry y" << std::endl;
      } else if (loopType == 2) {
        vec = &odometry_vector.theta;
        //std::cout << "odometry theta" << std::endl;
      } else if (loopType == 3) {
        vec = &odom_vector.x;
        //std::cout << "odometry y" << std::endl;
      } else if (loopType == 4) {
        vec = &odom_vector.y;
        //std::cout << "odometry y" << std::endl;
      } else if (loopType == 5) {
        vec = &odom_vector.theta;
        //std::cout << "odometry y" << std::endl;
      } else if (loopType == 6) {
        //std::cout << odometry_vector.x.size() << std::endl;
        //std::cout << odometry_vector.y.size() << std::endl;
        //std::cout << odometry_vector.theta.size() << std::endl;
        ros::shutdown();

      }
      if (loopType < 6) {
        for (auto beg = (*vec).begin(); beg != (*vec).end(); beg++) {
          //std::cout << *beg << std::endl;
          averageValueVector[loopType] += *beg;
        }
        averageValueVector[loopType] /= vec->size();

      loopType++;
      }
    }
  }
  auto names_beg = names.begin();
  for (auto beg = averageValueVector.begin(); beg != averageValueVector.end(); beg++) {
    if (names_beg != names.end()) {
      std::cout << *names_beg << std::endl;
      names_beg++;
    }
    std::cout << *beg << std::endl;
  }

  return 0;
}