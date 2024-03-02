#include <ros/ros.h>
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher broad_pub = n.advertise<std_msgs::String>("broad", 1000);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    if (chatter_pub.getNumSubscribers() > 0) {
      std_msgs::String msg;

      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("chatter %s", msg.data.c_str());

      chatter_pub.publish(msg);
      ++count;
    } else {
        ROS_INFO("NO chatter subscribers");
    }

    if (broad_pub.getNumSubscribers() > 0) {
      std_msgs::String msg;

      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("broad %s", msg.data.c_str());

      broad_pub.publish(msg);
      ++count;
    } else {
        ROS_INFO("NO broad_pub subscribers");
    }

    ros::spinOnce();

    loop_rate.sleep();
   
  }


  return 0;
}