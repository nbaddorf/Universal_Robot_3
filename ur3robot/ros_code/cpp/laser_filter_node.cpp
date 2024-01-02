#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

// std::vector<float>

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->ranges.size());
    // std::vector<float> laser_array
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_filter");

    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("scan", 10, laserCallback);

    ros::spin();

    return 0;
}