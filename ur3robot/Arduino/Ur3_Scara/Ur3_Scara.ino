#include "AS5600.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

#define DEBUG

//create ros msgs
ros::NodeHandle nh;

const int stepper_axis_1_enable_pin = 7;
const int stepper_axis_1_dir_pin = 8;
const int stepper_axis_1_step_pin = 9;

const int LED = 13;

// timers for the sub-main loop
unsigned long currentMillis;
unsigned long previousMillis = 0;  // set up timers
const float loopTime = 10; // 10


void setup() {

  #ifdef DEBUG
    Serial.begin(115200);
  #else
    // set baud rate to 115200
    nh.getHardware()->setBaud(115200);
    nh.initNode();  // init ROS
  #endif

  //Define axis 1 stepper pins
  pinMode(stepper_axis_1_enable_pin, OUTPUT);
  pinMode(stepper_axis_1_dir_pin, OUTPUT);
  pinMode(stepper_axis_1_step_pin, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  // put your main code here, to run repeatedly:
  #ifndef DEBUG
  nh.spinOnce();  // make sure we listen for ROS messages and activate the callback if there is one
  #endif

  currentMillis = millis();

  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    //Check if arm is connected to ROS and if not connected, turn off arm
    #ifndef DEBUG
      if (!nh.connected()) {
        digitalWrite(LED, HIGH);
        
      } else {
        digitalWrite(LED, LOW);
      }
    #endif

  }

}
