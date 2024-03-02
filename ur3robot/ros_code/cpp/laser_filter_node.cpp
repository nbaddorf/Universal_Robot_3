#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

ros::Publisher laser_pub;

std::vector<float> laser_lookup_table;
const int laser_ranges_num = 760;

static double d2r(double d) {
  static const auto PI = std::acos(-1);
  return (d / 180.0) * PI;
}

static double r2d(double d) {
  static const auto PI = std::acos(-1);
  return (d * 180.0) / PI;
}

void create_lookup_table(std::vector<float>(&lookup_table), int num_of_elements,
                         double robot_width) {
  double deg_conversion = 360.0 / num_of_elements;
  double half_robot_width = robot_width / 2.0;
  for (int i = 0; i < num_of_elements; i++) {
    double deg = i * deg_conversion;

    float distance;
    if (deg <= 45 || deg > 315 || (deg > 135 && deg <= 225)) {
      distance = half_robot_width / std::cos(d2r(deg));
    } else {
      distance = half_robot_width / std::sin(d2r(deg));
    }
    
    lookup_table.push_back(abs(distance));
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // There are 760 messages from the laser
  // ROS_INFO("I heard: [%lu]", msg->ranges.size()); //%l = long, %u = unsigned
  if (laser_pub.getNumSubscribers() > 0) {
    sensor_msgs::LaserScan filtered_msg;

    filtered_msg.header = msg->header;
    filtered_msg.angle_min = msg->angle_min;
    filtered_msg.angle_max = msg->angle_max;

    filtered_msg.time_increment = msg->time_increment;
    filtered_msg.angle_increment = msg->angle_increment;

    filtered_msg.scan_time = msg->scan_time;

    filtered_msg.range_min = msg->range_min;
    filtered_msg.range_max = msg->range_max;

    filtered_msg.intensities = msg->intensities;

    std::vector<float> filtered_ranges;

    for (int i = 0; i < laser_ranges_num; i++) {
      if (msg->ranges[i] >= laser_lookup_table.at(i)) {
        filtered_ranges.push_back(msg->ranges[i]);
      } else {
        filtered_ranges.push_back(0.0);
      }
    }
    filtered_msg.ranges = filtered_ranges;
    laser_pub.publish(filtered_msg);
  }
}

int main(int argc, char **argv) {
  double robot_width = 0.4191;  // my ur3 robot is 0.4191 meters width
  double inflation = 0.05;
  robot_width = robot_width + inflation;

  create_lookup_table(laser_lookup_table, laser_ranges_num, robot_width);

  ros::init(argc, argv, "laser_filter");

  ros::NodeHandle n;

  laser_pub = n.advertise<sensor_msgs::LaserScan>("scan/filtered", 10);
  ros::Subscriber laser_sub = n.subscribe("scan", 10, laserCallback);

  /*
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    // maybe add code to stop the laser if nothing is subscribed	//if
    // (laser_pub.getNumSubscribers() > 0) {

    //}
  }
  */

  ros::spin();

  return 0;
}