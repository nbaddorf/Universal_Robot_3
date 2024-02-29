#include <ros/ros.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>


using std::vector;

struct odomStruct {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
};



odomStruct *odometryPtr;
odomStruct *odomPtr;
int *loopNumPtr;


void odometry_sub_callback(const nav_msgs::Odometry &msg) {
  
  if ((*odometryPtr).x.size() < *loopNumPtr) { //dereferencing the ptr then using its structure
    //odometryPtr->x.push_back(msg.pose.pose.position.x); // same as above, but the -> means to dereference the ptr and get the structure
    //(*odometryPtr).y.push_back(msg.pose.pose.position.y);
    odometryPtr->x.push_back(msg.twist.twist.linear.x); 
    (*odometryPtr).y.push_back(msg.twist.twist.linear.y);
    odometryPtr->theta.push_back(msg.twist.twist.angular.z);// same as above, but the -> means to dereference the ptr and get the structure
  } /* else {
    std::cout << "x values: " << std::endl;
    for (auto xiter : (*odometryPtr).x) {
      std::cout << xiter << std::endl;
    }
    std::cout << "y values: " << std::endl;
    for (auto yiter : (*odometryPtr).y) {
      std::cout << yiter << std::endl;
    }
    std::cout << "calling shutdown " << std::endl;
    std::cout << (*odometryPtr).x.size() << std::endl;
    ros::shutdown();
    std::cout << (*odometryPtr).x.size() << std::endl;
  }
  */
  
}

void odom_sub_callback(const nav_msgs::Odometry &msg) {
  
  if ((*odomPtr).x.size() < *loopNumPtr) { //dereferencing the ptr then using its structure
    //odometryPtr->x.push_back(msg.pose.pose.position.x); // same as above, but the -> means to dereference the ptr and get the structure
    //(*odometryPtr).y.push_back(msg.pose.pose.position.y);
    odomPtr->x.push_back(msg.twist.twist.linear.x); 
    (*odomPtr).y.push_back(msg.twist.twist.linear.y);
    odomPtr->theta.push_back(msg.twist.twist.angular.z);// same as above, but the -> means to dereference the ptr and get the structure
  } 
}

/*
void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_analyzer");

  ros::NodeHandle n;
  //signal(SIGINT, mySigintHandler);
  odomStruct odometry_vector;
  odometryPtr = &odometry_vector;

  odomStruct odom_vector;
  odomPtr = &odom_vector;

  int loopNumber = 100;
  loopNumPtr = &loopNumber;

  //pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("laser_scan_macher/pose_stamped", 10);
  ros::Subscriber odometry_sub = n.subscribe("odometry/filtered", 10, odometry_sub_callback);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odom_sub_callback);

  ros::Rate loop_rate(10);

  unsigned loopType = 0;
  vector<double> averageValueVector(6,0);
  vector<std::string> names = {"odometry x: ", "odometry y: ", "odometry theta: ", "odom x: ", "odom y: ", "odom theta: "};

  while (ros::ok()) {
    if (odometry_vector.x.size() < loopNumber || odom_vector.x.size() < loopNumber) {
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