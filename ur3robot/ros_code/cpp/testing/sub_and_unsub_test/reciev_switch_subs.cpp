#include <ros/ros.h>
#include "std_msgs/String.h"
//#include <string>
#include <iostream>

std_msgs::String recieveMsg;
//decltype(ros::NodeHandle) *nhPtr;
//ros::Subscriber sub;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  /*
  recieveMsg.data = msg->data.c_str();  The c_str() function returns the string as a  ptr to char with null character.
  My if statement below works with or without it.
  */
  recieveMsg.data = msg->data; //.c_str();
  std::cout << recieveMsg.data << std::endl;
/*
  if (recieveMsg.data == "hello world 37") {
      ROS_INFO("WORKING");
      sub.shutdown();
      //loop_rate.sleep();
      ros::Subscriber sub = n.subscribe("broad", 5, chatterCallback);
      recieveMsg.data == "";
    }
    */
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  //nhPtr = n;
  

  ros::Subscriber sub = n.subscribe("chatter", 5, chatterCallback);
  ros::Rate loop_rate(10);
  
  //ros::spin();
  bool switched = false;
  while(ros::ok()) {
    ros::spinOnce();
    if (recieveMsg.data == "hello world 37" && !switched) {
      ROS_INFO("WORKING");
      switched = true;
      //sub.shutdown();
      //loop_rate.sleep();
      sub = n.subscribe("broad", 5, chatterCallback);
      recieveMsg.data == "";
    }
    loop_rate.sleep();
  }

  return 0;
}