#include <ros/ros.h>
#include "std_msgs/String.h"
//#include <string>
#include <iostream>

ros::Publisher second_pub;
bool isSubed = false;


//decltype(ros::NodeHandle) *nhPtr;
//ros::Subscriber sub;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  /* Cant figure out how to get nh pointer to work
  if (nhPtr) {
    if (msg->data == "hello world 37") {
       sub = *nhPtr.subscribe("chatter", 5, chatterCallback);
    }
  }
  */
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;  
  //nhPtr = &n;

  ros::Subscriber sub;
  second_pub = n.advertise<std_msgs::String>("secondPub", 1000);
  ros::Rate loop_rate(10);
  
  
  while(ros::ok()) {

    
    if (second_pub.getNumSubscribers() > 0 && !isSubed) {
        ROS_INFO("STARTING");
        sub = n.subscribe("chatter", 5, chatterCallback);
        isSubed = true;
    } else if (second_pub.getNumSubscribers() == 0 && isSubed) {
        sub.shutdown();
        ROS_INFO("shutting down");
        isSubed = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}