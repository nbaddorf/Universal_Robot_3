#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <boost/scoped_ptr.hpp>
#include "boost/thread.hpp"
#include <ros/ros.h>
#include <vector>
#include <string>

class MyRobot : public hardware_interface::RobotHW 
{
    public:
        MyRobot(ros::NodeHandle& nh);
        ~MyRobot();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        ros::Publisher command_pub;
        geometry_msgs::Point command_msg;
        
    protected:
 
        void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

        hardware_interface::JointStateInterface joint_state_interface_;
        //hardware_interface::EffortJointInterface effort_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        //joint_limits_interface::JointLimits limits;
        //joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        //joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        
        double joint_position_[3];
        double joint_velocity_[3];
        double joint_effort_[3];
        double joint_effort_command_[3];
        double joint_position_command_[3];
        
        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        ros::Subscriber joint_state_sub_;

        // This pointer is set from the ROS thread.
        sensor_msgs::JointState::ConstPtr feedback_msg_;
        boost::mutex feedback_msg_mutex_;
};
