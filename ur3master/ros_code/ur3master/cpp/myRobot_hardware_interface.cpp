#include "include/myRobot_hardware_interface.h"



MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {
 
// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    command_pub = nh_.advertise<geometry_msgs::Point>("/ur3/scara/arm_command",10);
    
//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);

    
}

MyRobot::~MyRobot() {
}

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void MyRobot::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    /*for (int i = 0; i < 3; i++) {
        //string joint_name = msg.name[i];
        std::vector<std::string> joint_names = msg->name;
        ROS_INFO("%c", joint_names[0].c_str());
    }
    */
    //joint_position_[0] = msg.position;
  //joint_position_[1] = 0.0;
  //joint_position_[2] = 0.0;

   boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

void MyRobot::init() {
        

    hardware_interface::JointStateHandle jointStateHandleA("base_link_to_tower", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandleA);
    
// Create effort joint interface as JointA accepts effort command.
    hardware_interface::JointHandle jointPositionHandleA(jointStateHandleA, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandleA); 

    

    hardware_interface::JointStateHandle jointStateHandleB("tower_to_arm1", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandleB);
    
// Create effort joint interface as JointB accepts effort command..
    hardware_interface::JointHandle jointPositionHandleB(jointStateHandleB, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandleB);
   
// Create Joint Limit interface for JointB
    //joint_limits_interface::getJointLimits("JointB", nh_, limits);
    //joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleB(jointEffortHandleB, limits);
    //effortJointSaturationInterface.registerHandle(jointLimitsHandleB);    
    
// Create joint_state_interface for JointC
    hardware_interface::JointStateHandle jointStateHandleC("arm1_to_arm2", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandleC);
// Create position joint interface as JointC accepts position command.
    hardware_interface::JointHandle jointPositionHandleC(jointStateHandleC, &joint_position_command_[2]);
    position_joint_interface_.registerHandle(jointPositionHandleC);
// Create Joint Limit interface for JointC
    //joint_limits_interface::getJointLimits("JointC", nh_, limits);
    //joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
    //positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    

// Register all joints interfaces 
  
    registerInterface(&joint_state_interface_);
    //registerInterface(&effort_joint_interface_);
   
    registerInterface(&position_joint_interface_);
    //registerInterface(&effortJointSaturationInterface);
    //registerInterface(&positionJointSaturationInterface);

    joint_state_sub_ = nh_.subscribe("/ur3/scara/joint_states", 1, &MyRobot::JointStateCallback, this);
   
}



void MyRobot::read() {
  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       
  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]
  //joint_position_[0] = 0.0;
  //joint_position_[1] = 0.0;
  //joint_position_[2] = 0.0;
/*
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    for (int i = 0; i < 4; ++i)
    {
*/

boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    for (int i = 0; i < 3; i++) {
        joint_position_[i] = feedback_msg_->position[i];
    }
    //joint_position_[]
  }
  
}

void MyRobot::write(ros::Duration elapsed_time) {
  // Safety
  //effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
  //positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 
  //joint_position_command_ for JointC.
  //ROS_INFO("I heard: [%d]", joint_position_command_[0]);
  //joint_position_[0] = joint_position_command_[0];
  //joint_position_[1] = joint_position_command_[1];
  //joint_position_[2] = joint_position_command_[2];

  //command_msg.x = joint_position_command_[0];
  //command_msg.z = joint_position_command_[1];
  command_msg.y = joint_position_command_[2];

  command_pub.publish(command_msg);
}

int main(int argc, char** argv)
{
    //Initialze the ROS node.
    ros::init(argc, argv, "MyRobot_hardware_inerface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2); 
    
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    //ros::spin();
    
    return 0;
}