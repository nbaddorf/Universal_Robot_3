#include <ros/ros.h>
#include "std_msgs/String.h"
//#include <string>
#include <iostream>

std_msgs::String recieveMsg;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  /*
  recieveMsg.data = msg->data.c_str();  The c_str() function returns the string as a  ptr to char with null character.
  My if statement below works with or without it.
  */
  recieveMsg.data = msg->data; //.c_str();
  std::cout << recieveMsg.data << std::endl;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 5, chatterCallback);
  ros::Rate loop_rate(10);

  /*
  ros::Subscriber sub = n.subscribe("chatter", 5, chatterCallback);
  ros::Rate loop_rate(5);

  // //when running reciever loop slower than broadcaster the sub checker is only 
  //run 5 times a second , but if the subscriber queue is greater than 1 and there is multiple messages it 
  //calls the callback twice because of the extra messages. If the subscriber queue is 1 then it just runs the callback
  //5 times per loop, and misses messages.

  //IF queue is greater than 1, when ros::spinOnce runs it will call the callback twice so 
  //if you are trying to do an if statement in the ros::ok loop, you will miss messages.
  //The below didnt work because the data in recieveMsg at the time of checking it was
  //(..34, ..36, ..38, ...) To fix this, increate the loop rate so you end up calling the 
  //callback only once per message.

  */
  

  while(ros::ok()) {
    ros::spinOnce();
    if (recieveMsg.data == "hello world 37") {
      ROS_INFO("WORKING");
      //sub.shutdown();
      //loop_rate.sleep();
      ros::Subscriber sub = n.subscribe("broad", 5, chatterCallback);
      recieveMsg.data == "";
    }
    loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}