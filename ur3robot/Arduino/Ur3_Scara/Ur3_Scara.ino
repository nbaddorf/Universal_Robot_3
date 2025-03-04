#include "AS5600.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <FlexCAN_T4.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3.h>

//#define DEBUG
//#define PID_TUNE

/*
Axis 3: offset = 2.6279
position after home: -2.6
other end stop: 2.4
*/

//create ros msgs
ros::NodeHandle nh;

/* // FOR OLD STEPPER DRIVERS
const int stepper_axis_1_enable_pin = 7; //tower axis
const int stepper_axis_1_dir_pin = 8;
const int stepper_axis_1_step_pin = 9;

const int stepper_axis_2_enable_pin = 4; //Z axis
const int stepper_axis_2_dir_pin = 5;
const int stepper_axis_2_step_pin = 6;

const int limit_1_pin = 15;
//These are the xlr ports on top
const int limit_2_pin = 21;  //axis 2 limit switch but labled limit 1 on top ports
const int limit_3_pin = 20;

// Define a stepper and the pins it will use
AccelStepper axis1(1, stepper_axis_1_step_pin, stepper_axis_1_dir_pin); //tower
AccelStepper axis2(1, stepper_axis_2_step_pin, stepper_axis_2_dir_pin); //Z axis
*/ 

// homing button pin
const int homing_button_pin = 17;

//E_Stop variables
const int e_stop_pin = 16;

bool e_stop = true;
bool old_e_stop = true;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
//CAN_message_t msg;

const int LED = 13;

//top speeds and accelerations for axis:
double axis1_top_speed = 0.75; //1 radian per second
double axis1_acceleration = 0.5; //1 radian per second per second

double axis2_top_speed = 6; //6 mm per second
double axis2_acceleration = 2; //2 mm per second per second

double axis3_top_speed = 0.5; //2 * 3.1415; //1 radian per second (temp 7.8125 rad per second)
double axis3_acceleration = 1; //1 radian per second per second

//math for top speeds and accelerations:
const int axis1_steps_per_motor_rev = 800;
const int axis1_gear_ratio = 8; //20:160 teeth
const double axis1_steps_per_rad = (axis1_steps_per_motor_rev * axis1_gear_ratio) / (2.0 * PI); //1018.59 steps per rad

const int axis2_steps_per_motor_rev = 200; //Z axis
const int axis2_gear_ratio = 30; //30:1 worm drive, 20:1 belt drive = 40mm per rev of pulley (CHECK)
const double axis2_mm_per_rad = 0.212207; // mm

const int axis3_gear_ratio = 4; //20:80 teeth
//double axis3_position_old = 0;

// timers for the sub-main loop
unsigned long currentMillis;
unsigned long previousMillis = 0;  // set up timers
const float loopTime = 10; // 10
unsigned long previousPIDMillis = 0;  // set up timers
const float PIDloopTime = 40; //was 30

#ifdef DEBUG
  bool ros_connected = true;
#else 
  bool ros_connected = false;
#endif

struct {
  double x = 0;
  double y = -2.6;
  double z = 0.003;
} arm_position; // structure with name arm_position.

bool has_homed = false; // SHOULD BE false
bool started_homing = false;

struct {
  bool axis1 = false;
  bool axis2 = false;
  bool axis3 = false;
  //const double axis1_homing_speed = 0.12;
  //const int axis2_homing_speed = 3;
  //const int axis3_homing_speed = 10;
  const double axis1_offset = 4.63239; //encoder offset in rad
  const double axis2_offset = 0;
  const double axis3_offset = 2.6279; // encoder offset in rad 0.09052
} homed;

struct can_motor {
  uint8_t id = 0;
  bool sent_home_command = false;
  bool motor_enabled = true;
  //bool sent_get_encoder_command = false;
  uint8_t request_encoder_counter = 0;
  bool sent_position_command = false;
  double encoder = 0; // Value to store encoder position.
  double lower_limit = 0; // Lower limit of movement allowed (in m or rad) (Used in constrain for input value)
  double upper_limit = 0;// Upper limit of movement allowed (in m or rad) 
  double pid_speed = 0; // Speed value that will be used for running motor at speed.
  double setpoint = 0; // Setpoint for PID value
};

can_motor axis1;
can_motor axis2;
can_motor axis3;

double axis2_current_pos = 0;

PID axis1PIDLoop(&axis1.encoder, &axis1.pid_speed, &axis1.setpoint, 1700, 1, 0.0, REVERSE);
PID axis2PIDLoop(&axis2_current_pos, &axis2.pid_speed, &axis2.setpoint, 28000, 1, 0.0, REVERSE);
PID axis3PIDLoop(&axis3.encoder, &axis3.pid_speed, &axis3.setpoint, 1100, 2.0, 0.0, REVERSE);

//char robot_id = "";
char *joint_name[4] = {"base_link_to_tower", "tower_to_arm1", "arm1_to_arm2", "kinect_rotator_to_kinect_base"};
float joint_pos[4];
//float vel[6];
//float eff[6];

#ifdef PID_TUNE
  void pidSet(const geometry_msgs::Vector3& vel) {
    double p=vel.x;
    double i=vel.y;
    double d=vel.z;
    axis3PIDLoop.SetTunings(p,i,d);
  }
#endif

void pointCallback(const geometry_msgs::Point& point) {
  //arm_position.x = constrain(point.x, axis1.lower_limit, axis1.upper_limit);
  //arm_position.y = constrain(point.y, axis3.lower_limit, axis3.upper_limit); 
  //arm_position.z = constrain(point.z, axis2.lower_limit, axis2.upper_limit);//505 mm total movement I think this is correct
  arm_position.x = constrain(point.x, axis1.lower_limit, axis1.upper_limit);
  arm_position.y = constrain(point.y, axis3.lower_limit, axis3.upper_limit); 
  arm_position.z = constrain(point.z, axis2.lower_limit, axis2.upper_limit);
}

ros::Subscriber<geometry_msgs::Point> pointSub("ur3/scara/arm_command", pointCallback);
//ros::Subscriber<sensor_msgs::JointState> joinstates_sub("ur3/scara/arm_command", jointstates_callback);

sensor_msgs::JointState scara_joints;
ros::Publisher joint_pub("ur3/scara/joint_states", &scara_joints);

#ifdef PID_TUNE
  ros::Subscriber<geometry_msgs::Vector3> pidSub("set_pid", pidSet);
  geometry_msgs::Vector3 pid_helper;
  ros::Publisher pidHelper("pid_helper", &pid_helper);
#endif

void setup() {

  #ifdef DEBUG
    Serial.begin(115200);
  #else
    // set baud rate to 115200
    nh.getHardware()->setBaud(115200);
    nh.initNode();  // init ROS

    nh.subscribe(pointSub);
    //nh.subscribe(joinstates_sub);
    nh.advertise(joint_pub);
    #ifdef PID_TUNE
      nh.subscribe(pidSub);
      nh.advertise(pidHelper);
    #endif
  #endif

  can1.begin();
  can1.setBaudRate(500000); //250000

  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canRecieve);
  can1.mailboxStatus();

  axis1PIDLoop.SetMode(AUTOMATIC);
  axis1PIDLoop.SetOutputLimits(-250, 250);
  axis1PIDLoop.SetSampleTime(PIDloopTime);

  axis2PIDLoop.SetMode(AUTOMATIC);
  axis2PIDLoop.SetOutputLimits(-200, 200);
  axis2PIDLoop.SetSampleTime(PIDloopTime);

  axis3PIDLoop.SetMode(AUTOMATIC);
  axis3PIDLoop.SetOutputLimits(-250, 250);
  axis3PIDLoop.SetSampleTime(PIDloopTime);

  pinMode(LED, OUTPUT);
  pinMode(homing_button_pin, INPUT_PULLUP);
  pinMode(e_stop_pin, INPUT_PULLUP);
  
  //pinMode(limit_2_pin, INPUT); //Limit port 1 on top of robot
  //pinMode(limit_3_pin, INPUT); //Limit port 2 on top of robot

  axis1.id = 0x01; // This might not be working
  axis1.lower_limit = -4.600;
  axis1.upper_limit = 1.0;

  axis2.id = 0x02;
  axis2.lower_limit = 0.0;
  axis2.upper_limit = 0.5;

  axis3.id = 0x03;
  axis3.lower_limit = homed.axis3_offset * -1;
  axis3.upper_limit = 2.4;


  old_e_stop = digitalRead(e_stop_pin); //set e_stop values to whatever they currently are

  /* CHECK THIS TO MAKE SURE IT WORKS    */
  setCanEnable(axis1, !old_e_stop); //set motor to be enabled or not
  setCanEnable(axis2, !old_e_stop); //set motor to be enabled or not
  setCanEnable(axis3, !old_e_stop); //set motor to be enabled or not

}

void loop() {
  // put your main code here, to run repeatedly:
  #ifndef DEBUG
  nh.spinOnce();  // make sure we listen for ROS messages and activate the callback if there is one
  #endif

  can1.events();

  currentMillis = millis();

  if (currentMillis - previousPIDMillis >= PIDloopTime) {  // run a loop every 30ms
    previousPIDMillis = currentMillis;
    getCanEncoderVal(axis1);
    getCanEncoderVal(axis2);
    getCanEncoderVal(axis3);
    axis1.setpoint = arm_position.x; // in rad
    axis2.setpoint = arm_position.z; // in m
    axis3.setpoint = arm_position.y; // in rad

    if (!e_stop && ros_connected) {
    if (has_homed) {
      axis1PIDLoop.Compute();
      axis2PIDLoop.Compute();
      axis3PIDLoop.Compute();
    }
  }

  if (has_homed && !e_stop) {
      setCanMotorSpeed(axis1, axis1.pid_speed, 255);
      setCanMotorSpeed(axis2, (abs(axis2.pid_speed) <= 2) ? 0 : axis2.pid_speed, 255);
      setCanMotorSpeed(axis3, axis3.pid_speed, 255);
      //Serial.println((abs(axis3.setpoint - axis3.encoder) <= 0.001) ? 0 : axis3.pid_speed);
      //setCanMotorSpeed(axis3, (abs(axis3.setpoint - axis3.encoder) <= 0.001) ? 0 : axis3.pid_speed, 255);
    }
  }


  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    e_stop = digitalRead(e_stop_pin); // if estop is pressed then value is true

    if (!has_homed && !e_stop) { // check if we havent homed yet, and the estop is not active
      if (!digitalRead(homing_button_pin)) { //if homing button pressed then start homing sequence
        started_homing = true;
      } else if (started_homing) {
        home_axis();
      }
    }

    if (e_stop != old_e_stop) { //if estop is activated
      if (e_stop) {

        axis1PIDLoop.SetMode(MANUAL);
        setCanMotorSpeed(axis1, 0, 255);
        setCanEnable(axis1, false); //disable motor

        axis2PIDLoop.SetMode(MANUAL);
        setCanMotorSpeed(axis2, 0, 255);
        setCanEnable(axis2, false); //disable motor

        axis3PIDLoop.SetMode(MANUAL);
        setCanMotorSpeed(axis3, 0, 255);
        setCanEnable(axis3, false); //disable motor

      } else {
        axis1PIDLoop.SetMode(AUTOMATIC);
        setCanEnable(axis1, true);

        axis2PIDLoop.SetMode(AUTOMATIC);
        setCanEnable(axis2, true);

        axis3PIDLoop.SetMode(AUTOMATIC);
        setCanEnable(axis3, true);
      }
    }


    //Check if arm is connected to ROS and if not connected, turn off arm
    #ifndef DEBUG
      if (!nh.connected()) {
        digitalWrite(LED, HIGH);
        setCanMotorSpeed(axis1, 0, 0);
        setCanMotorSpeed(axis2, 0, 0);
        setCanMotorSpeed(axis3, 0, 0);
        ros_connected = false;
      } else {
        digitalWrite(LED, LOW);
        ros_connected = true;
      }
    #endif

    /*
    Serial.print("axis1: ");
    Serial.println(axis1.encoder);
    Serial.print("axis2: ");
    Serial.println(axis2.encoder);
    Serial.print("axis3: ");
    Serial.println(axis3.encoder);
    */

    #ifndef DEBUG

    #ifdef PID_TUNE
      pid_helper.y = arm_position.y;
      pid_helper.z = axis3.pid_speed;
      pidHelper.publish(&pid_helper);
    #endif


      joint_pos[0] = axis1.encoder; // tower axis
      joint_pos[1] = axis2_current_pos; //z axis
      joint_pos[2] = axis3.encoder; // arm2 axis
      joint_pos[3] = 0.0; //kinect rotator

      scara_joints.name_length = 4;
      scara_joints.position_length = 4;

      scara_joints.header.stamp = nh.now();
      //scara_joints.header.frame_id = robot_id;
      scara_joints.name = joint_name;
      scara_joints.position = joint_pos;

      joint_pub.publish( &scara_joints);
    #endif

    old_e_stop = e_stop;

  }
}

void canRecieve(const CAN_message_t &msg) {
  #ifdef DEBUG
  
  if (msg.buf[0] != 0x31) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
  }
  #endif

  switch (msg.buf[0]) {
      case 0x31: // encoder
        switch (msg.id) {
          case 1:
            axis1.encoder = (returnEncoderRad(msg) / axis1_gear_ratio) - homed.axis1_offset;
            axis1.encoder = (axis1.encoder > axis1.upper_limit + 5) ? axis1.lower_limit : axis1.encoder;
            //axis1.sent_get_encoder_command = false;
            axis1.request_encoder_counter = 0;
            //Serial.print("axis1: ");
            //Serial.println(axis1.encoder);
            break;
          case 2:
            axis2.encoder = 1647099.33 - (returnEncoderRad(msg)) - homed.axis2_offset; //54903.31 is the vars overflow.
            axis2.encoder = (axis2.encoder > 2500) ? 0.0 : axis2.encoder;
            axis2_current_pos = (axis2.encoder * axis2_mm_per_rad) / 1000;
            //axis2.sent_get_encoder_command = false;
            axis2.request_encoder_counter = 0;
            //Serial.print("axis2: ");
            //Serial.println(axis2.encoder);
            break;
          case 3:
            axis3.encoder = (returnEncoderRad(msg) / axis3_gear_ratio) - homed.axis3_offset;
            axis3.encoder = (axis3.encoder > 6) ? axis3.lower_limit : axis3.encoder;
            //axis3.sent_get_encoder_command = false;
            axis3.request_encoder_counter = 0;
            //Serial.print("axis3: ");
            //Serial.println(axis3.encoder, 5);
            
            break;
        }
        break;
      case 0x91: //homing responce (If CanRSP is on)
        switch (msg.id) {
          case 1:
            homed.axis1 = (msg.buf[1] == 2);
            axis1.sent_home_command = !(msg.buf[1] == 2);
            break;
          case 2:
            homed.axis2 = (msg.buf[1] == 2);
            axis2.sent_home_command = !(msg.buf[1] == 2);
            break;
          case 3:
            homed.axis3 = (msg.buf[1] == 2);
            axis3.sent_home_command = !(msg.buf[1] == 2);
            break;
        }
        break;
      case 0xF1: //get motor status
        switch (msg.id) {
          case 1:
          
            break;
          case 2:

            break;
          case 3:
          
            break;
        }
        break;
      case 0xF5: //return from motor in absolute position command
      
        break;
      }
}

void goHomeCan(can_motor &motor) {
  if (!motor.sent_home_command) {
  CAN_message_t msgSend;
  msgSend.len=2;
  msgSend.id= motor.id;
  msgSend.buf[0] = 0x91; //go home command
  msgSend.buf[1] = msgSend.buf[0] + msgSend.id; //crc

  can1.write(msgSend);
  motor.sent_home_command = true;
  }


}

void runCanToPosition(can_motor &motor, double rad, double speed, uint8_t acc) { 
  if (!motor.sent_position_command) {
    CAN_message_t msgSend;
    speed = constrain(speed, 0, 9.00);
    //speed calc (rad/s to rev/min):
    // speed = (speed / 2PI) gives us motor revs per second at 16 subdivisions
    // speed = (speed / 2PI) * 4 gives us arm revs per second at 16 subdivision
    // speed = (speed / 2PI) * 4 * 60 gives us arm revs per min at 16 subdivison
    // speed = (speed / 2PI) * 4 * 60 * 8 gives us arm revs per min at 128 subdivison
    uint16_t speedRev = ((speed * 2.00) / PI) * 60.00 * 8.00; //convert input of radian/second to revolution/min
    //0x4000 = 1 motor rev. * 4 = 1 arm rev
    //motor max range is 320 deg. 651 is centered
    long encoder_counts = ((0x4000 * 2) / PI) * ( (constrain(rad * 100, motor.lower_limit * 100, motor.upper_limit * 100) / 100) + homed.axis3_offset);//((0x4000 * 4) * 2) / PI; //0x10000 at 64 subdivision is 1 motor rev
    //Serial.println(encoder_counts); // should be about 32768 (0x8001) with rad = PI
    msgSend.len = 8;
    msgSend.id = motor.id;
    msgSend.buf[0] = 0xF5; //run motor to absolute position mode 4
    msgSend.buf[2] = speedRev; 
    msgSend.buf[1] = speedRev >> 8; 
    msgSend.buf[3] = acc; //acceleration
    msgSend.buf[6] = encoder_counts;
    msgSend.buf[5] = encoder_counts >> 8;
    msgSend.buf[4] = encoder_counts >> 16;

    uint8_t crc = msgSend.id;
    for (int i = 0; i<7; i++) {
      crc += msgSend.buf[i];
    }

    msgSend.buf[7] = crc;
    
    can1.write(msgSend);
    motor.sent_position_command = true;
  }
}

void setCanMotorSpeed(can_motor &motor, short speed, uint8_t acc) { //DOESNT WORK
    CAN_message_t msgSend;
    msgSend.len = 5;
    msgSend.id = motor.id;

    //4095 is the max size for unsigned 12 bit number
    //motor limit is +-3000 RPM (this might be at 16 subdivisions I dont know)
    uint16_t bin_speed = abs(constrain(speed, -3000, 3000)); // make an uint16 where the lowest 12 bits get set regardless of signedness, and the high-order 4 bits are allways 0. Example for value 4095 (or -4095) value is 0000111111111111.
    bin_speed = (speed < 0) ? bin_speed | (1U << 15) : bin_speed; // if speed is negative then perform bitwise or. 1U == unsigned int (16 bit minimum) with a value of 1. (0000000000000001). Then left shift it 15 places to result in 1000000000000000.

    msgSend.buf[0] = 0xF6; //run motor in speed mode
    msgSend.buf[1] = bin_speed >> 8; // shift the high-order bits 8 positions down.
    msgSend.buf[2] = bin_speed; // sets the 8 bit num to the lowest order 8 bits in our 16 bit number
    msgSend.buf[3] = acc; //acceleration

    uint8_t crc = msgSend.id;
    for (int i = 0; i<7; i++) {
      crc += msgSend.buf[i];
    }

    msgSend.buf[4] = crc;
    can1.write(msgSend);

}

void setCanEnable(can_motor &motor, bool enabled) {
  CAN_message_t msgSend;
  msgSend.len=3;
  msgSend.id= motor.id;
  msgSend.buf[0] = 0xF3; //motor enable command
  msgSend.buf[1] = enabled ? 0x01 : 0x00; // weather do enable or disable motor
  msgSend.buf[2] = motor.id + msgSend.buf[0] + msgSend.buf[1]; //crc
  motor.motor_enabled = enabled;

  can1.write(msgSend);
}

void setEStop (can_motor &motor) { //This disables the motor and I think you have to power cycle motor
  CAN_message_t msgSend;
  msgSend.len=2;
  msgSend.id= motor.id;
  msgSend.buf[0] = 0xF7; //motor enable command
  msgSend.buf[1] = motor.id + msgSend.buf[0]; //crc

  can1.write(msgSend);
}

void getCanEncoderVal(can_motor &motor) {
  //if (!motor.sent_get_encoder_command) {
  if (motor.request_encoder_counter == 0) {
    CAN_message_t msgSend;
    msgSend.len = 2;
    msgSend.id = motor.id;
    msgSend.buf[0] = 0x31; //encoder mode
    msgSend.buf[1] = msgSend.id + msgSend.buf[0]; //checksum
    //motor.sent_get_encoder_command = true;
    can1.write(msgSend);
    motor.request_encoder_counter++;
  } else if (motor.request_encoder_counter > 3) {
    motor.request_encoder_counter = 0;
  } else {
    motor.request_encoder_counter++;
  }

  
}

double returnEncoderRad(CAN_message_t recievedMsg) {
  u_long encoderVal = recievedMsg.buf[2]; //demo 0x01;
  encoderVal = (encoderVal << 8) | recievedMsg.buf[3];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[4];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[5];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[6];

  return ((encoderVal / double (0x2000)) * PI); // take the encoder then divide by 1/2 rotation then multiply by PI
}

void setMotorStop (can_motor &motor) {
  CAN_message_t msgSend;
  msgSend.len=8;
  msgSend.id= motor.id;
  msgSend.buf[0] = 0xF5; //motor enable command
  msgSend.buf[1] = 0;
  msgSend.buf[2] = 0;
  msgSend.buf[3] = 40;
  msgSend.buf[4] = 0;
  msgSend.buf[5] = 0;
  msgSend.buf[6] = 0;
  
  uint8_t crc = msgSend.id;
  for (int i = 0; i<7; i++) {
    crc += msgSend.buf[i];
  }

  msgSend.buf[7] = crc;

  can1.write(msgSend);
}

void query_motor(can_motor &motor) {
CAN_message_t msgSend;
  msgSend.len=2;
  msgSend.id= motor.id;
  msgSend.buf[0] = 0xF1; //query motor command
  msgSend.buf[1] = motor.id + msgSend.buf[0]; //crc

  can1.write(msgSend);
}


void home_axis() {
  if (!homed.axis3) {
    //query_motor(axis3);
    goHomeCan(axis3);
  } else if (!homed.axis2) {
    //query_motor(axis2);
    goHomeCan(axis2);
  } else if (!homed.axis1) {
    //query_motor(axis1);
    goHomeCan(axis1);
  }
  if (homed.axis1 && homed.axis2 && homed.axis3) {
    has_homed = true;
    started_homing = false;
  }
  
}


