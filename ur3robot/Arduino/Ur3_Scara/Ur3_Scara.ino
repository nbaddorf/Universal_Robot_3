#include "AS5600.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <AccelStepper.h>
#include <math.h>

//#define DEBUG

//create ros msgs
ros::NodeHandle nh;

const int stepper_axis_1_enable_pin = 7;
const int stepper_axis_1_dir_pin = 8;
const int stepper_axis_1_step_pin = 9;

// Define a stepper and the pins it will use
AccelStepper axis1(1, stepper_axis_1_step_pin, stepper_axis_1_dir_pin);

const int LED = 13;

//top speeds and accelerations for axis:
double axis1_top_speed = 0.75; //1 radian per second
double axis1_acceleration = 0.5; //1 radian per second per second

//math for top speeds and accelerations:
const int axis1_steps_per_motor_rev = 800;
const int axis1_belt_ratio = 8; //20:160 teeth
const double axis1_steps_per_rad = (axis1_steps_per_motor_rev * axis1_belt_ratio) / (2.0 * PI); //509.29 steps per rad

bool axis1_enabled = false;

// timers for the sub-main loop
unsigned long currentMillis;
unsigned long previousMillis = 0;  // set up timers
const float loopTime = 10; // 10

//double positionArray[] = {0.0, 0.0, 0.0}; //x y z
struct {
  double x = 0;
  double y = 0;
  double z = 0;
} arm_position;

void pointCallback(const geometry_msgs::Point& point) {
  arm_position.x = constrain(point.x, -4, 4); //0.3
  arm_position.y = constrain(point.y, -0.3, 0.3);
  arm_position.z = constrain(point.z, -0.6, 0.6);
}

ros::Subscriber<geometry_msgs::Point> pointSub("arm_pos", pointCallback);

sensor_msgs::JointState scara_joints;
ros::Publisher joint_pub("odom", &odom_msg);


void setup() {

  #ifdef DEBUG
    Serial.begin(115200);
  #else
    // set baud rate to 115200
    nh.getHardware()->setBaud(115200);
    nh.initNode();  // init ROS

    nh.subscribe(pointSub);
  #endif

  //Define axis 1 stepper
  axis1.setEnablePin(stepper_axis_1_enable_pin);
  axis1.setMaxSpeed(axis1_top_speed * axis1_steps_per_rad); 
  axis1.setAcceleration(axis1_acceleration * axis1_steps_per_rad);
  axis1.setPinsInverted(false, false, true);

  pinMode(LED, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  // put your main code here, to run repeatedly:
  #ifndef DEBUG
  nh.spinOnce();  // make sure we listen for ROS messages and activate the callback if there is one
  #endif

  axis1.run();

  currentMillis = millis();

  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    //arm_position.x = 1.0;

    int axis1_position = arm_position.x * axis1_steps_per_rad;
    #ifdef DEBUG
      Serial.println(axis1_position);
    #endif
    axis1.moveTo(axis1_position);

    //Check if arm is connected to ROS and if not connected, turn off arm
    #ifndef DEBUG
      if (!nh.connected()) {
        digitalWrite(LED, HIGH);
        axis1.stop();        
      } else {
        digitalWrite(LED, LOW);
      }
    #endif

    if (!axis1.isRunning() && axis1_enabled) {
      axis1.disableOutputs();
      axis1_enabled = false;
    } else if (axis1.isRunning() && !axis1_enabled) {
      axis1.enableOutputs();
      axis1_enabled = true;
    }

  }
}
