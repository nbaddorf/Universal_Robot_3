#include "AS5600.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <AccelStepper.h>
#include <math.h>
#include <FlexCAN_T4.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3.h>

//#define DEBUG
#define PID_TUNE

//create ros msgs
ros::NodeHandle nh;

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

// homing button pin
const int homing_button_pin = 17;

const int e_stop_pin = 16;

bool e_stop = true;
bool old_e_stop = true;

// Define a stepper and the pins it will use
AccelStepper axis1(1, stepper_axis_1_step_pin, stepper_axis_1_dir_pin); //tower
AccelStepper axis2(1, stepper_axis_2_step_pin, stepper_axis_2_dir_pin); //Z axis

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
//CAN_message_t msg;

const int LED = 13;

//top speeds and accelerations for axis:
double axis1_top_speed = 0.75; //1 radian per second
double axis1_acceleration = 0.5; //1 radian per second per second

double axis2_top_speed = 6; //6 mm per second
double axis2_acceleration = 2; //2 mm per second per second

double axis3_top_speed = 0.5; //2 * 3.1415; //1 radian per second
double axis3_acceleration = 1; //1 radian per second per second

//math for top speeds and accelerations:
const int axis1_steps_per_motor_rev = 800;
const int axis1_belt_ratio = 8; //20:160 teeth
const double axis1_steps_per_rad = (axis1_steps_per_motor_rev * axis1_belt_ratio) / (2.0 * PI); //1018.59 steps per rad

const int axis2_steps_per_motor_rev = 200; //Z axis
const int axis2_gear_ratio = 30; //30:1 worm drive, 20:1 belt drive = 40mm per rev of pulley (CHECK)
const double axis2_steps_per_mm = (axis2_steps_per_motor_rev * axis2_gear_ratio) / 40.0; //150 steps per mm

const int axis3_gear_ratio = 4; //20:80 teeth
double axis3_position_old = 0;
int can_loop_counter = 0;


bool axis1_enabled = false;
bool axis2_enabled = false;

// timers for the sub-main loop
unsigned long currentMillis;
unsigned long previousMillis = 0;  // set up timers
const float loopTime = 10; // 10

#ifdef DEBUG
  bool ros_connected = true;
#else 
  bool ros_connected = false;
#endif

struct {
  double x = 0;
  double y = 2;
  double z = 0.02;
} arm_position; // structure with name arm_position.

bool has_homed = false;
bool started_homing = false;

struct {
  bool axis1 = false;
  bool axis2 = false;
  bool axis3 = false;
  const double axis1_homing_speed = 0.12;
  const int axis2_homing_speed = 3;
  const int axis3_homing_speed = 10;
  const double axis1_offset = ((axis1_steps_per_rad * 2 * PI) * (0.25)) + 25;
  const double axis2_offset = 0;
  const double axis3_offset = 0.09052;
} homed;

struct can_motor {
  uint8_t id = 0;
  bool sent_home_command = false;
  bool motor_enabled = true;
  bool sent_get_encoder_command = false;
  bool sent_position_command = false;
  double encoder = 0;
  double lower_limit = 0;
  double upper_limit = 0;
  double pid_speed = 0;
  double setpoint = 0;
};

can_motor axis3;
double test = 3;
PID axis3PIDLoop(&axis3.encoder, &axis3.pid_speed, &arm_position.y, 800, 0.001, 0.0, REVERSE);

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
  arm_position.x = constrain(point.x, -4.21239, 1.59534); //0.3
  arm_position.y = constrain(point.y, axis3.lower_limit, axis3.upper_limit); 
  arm_position.z = constrain(point.z * 100, 0.0, 0.5 * 100) / 100;//505 mm total movement I think this is correct
}
/*
void jointstates_callback(const sensor_msgs::JointState& jointState) {
  arm_position.x = constrain(jointState.position[0], -1.3708, 4.71239); //0.3
  arm_position.y = constrain(jointState.position[1], -2.7, 2.7); 
  arm_position.z = constrain(jointState.position[2] * 100, -0.25 * 100, 0.25 * 100) / 100;//505 mm total movement I think this is correct
}
*/


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

  //Testing 
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canRecieve);
  can1.mailboxStatus();

  //Define axis 1 stepper (Tower axis)
  axis1.setEnablePin(stepper_axis_1_enable_pin);
  axis1.setMaxSpeed(axis1_top_speed * axis1_steps_per_rad); 
  axis1.setAcceleration(axis1_acceleration * axis1_steps_per_rad);
  axis1.setPinsInverted(false, false, true);

   //Define axis 2 stepper (Z axis)
  axis2.setEnablePin(stepper_axis_2_enable_pin);
  axis2.setMaxSpeed(axis2_top_speed * axis2_steps_per_mm);
  axis2.setAcceleration(axis2_acceleration * axis2_steps_per_mm);
  axis2.setPinsInverted(false, false, true);

  axis3PIDLoop.SetMode(AUTOMATIC);
  axis3PIDLoop.SetOutputLimits(-250, 250);
  axis3PIDLoop.SetSampleTime(30);

  pinMode(LED, OUTPUT);
  pinMode(homing_button_pin, INPUT_PULLUP);
  pinMode(e_stop_pin, INPUT_PULLUP);
  pinMode(limit_1_pin, INPUT);
  pinMode(limit_2_pin, INPUT);
  pinMode(limit_3_pin, INPUT);

  axis3.id = 0x01;
  axis3.lower_limit = 0.0;
  axis3.upper_limit = 5.4;

  old_e_stop = digitalRead(e_stop_pin); //set e_stop values to whatever they currently are
  setCanEnable(axis3, !old_e_stop); //set motor to be enabled or not

}

void loop() {
  // put your main code here, to run repeatedly:

  // put your main code here, to run repeatedly:
  #ifndef DEBUG
  nh.spinOnce();  // make sure we listen for ROS messages and activate the callback if there is one
  #endif

  can1.events();

  if (!e_stop && ros_connected) {
    if (!has_homed && started_homing) {
      axis1.runSpeed();
      axis2.runSpeed();
    } else {
      axis1.run();
      axis2.run();
      axis3PIDLoop.Compute();
    }
  }


  currentMillis = millis();

  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    getCanEncoderVal(axis3);
    axis3.setpoint = arm_position.y;
    


    e_stop = digitalRead(e_stop_pin); // if estop is pressed then value is true

    if (!has_homed && !e_stop) { // check if we havent homed yet, and the estop is not active
      if (!digitalRead(homing_button_pin)) { //if homing button pressed then start homing sequence
        started_homing = true;
      } else if (started_homing) { //run homing sequence if started homing is true
        home_axis();
      }
    }

    if (e_stop != old_e_stop) { //if estop is activated reset active motors to needing homing
      if (e_stop) {

        //setMotorStop(axis3);
        //setCanEnable(axis3, false); //disable motor
        axis3PIDLoop.SetMode(MANUAL);
        setCanMotorSpeed(axis3, 0, 255);
        setCanEnable(axis3, false); //disable motor

        if (axis1.isRunning()) {
          homed.axis1 = false;
          axis1.moveTo(axis1.currentPosition()); //disable the motor
          axis1.setSpeed(0);
          has_homed = false;
          started_homing = false;
        }
        if (axis2.isRunning()) {
          homed.axis2 = false;
          axis2.moveTo(axis2.currentPosition()); // disable the motor
          axis2.setSpeed(0);
          has_homed = false;
          started_homing = false;
        }
      } else {
        axis3PIDLoop.SetMode(AUTOMATIC);
        setCanEnable(axis3, true);
        //setCanMotorSpeed(axis3, 10, 255);
      }
    }

    int axis1_position = arm_position.x * axis1_steps_per_rad; //Tower
    int axis2_position = arm_position.z * (axis2_steps_per_mm * 1000.00); //Z axis

    //arm_position.y = 0;

    if (has_homed && !e_stop) {
      axis1.moveTo(axis1_position);
      axis2.moveTo(axis2_position);
      //runCanToPosition(axis3, arm_position.y, axis3_top_speed, 200);
      
      setCanMotorSpeed(axis3, axis3.pid_speed, 255);
      
      /* if (can_loop_counter >= 9) {
      if (axis3_position_old != arm_position.y) {
        //if (can_loop_counter >= 9) {
          can_loop_counter = 0;
          runCanToPosition(axis3, arm_position.y, axis3_top_speed, 150);
          axis3_position_old = arm_position.y;
      }
      } else {
        can_loop_counter++;
        }
        */
    }

    //Check if arm is connected to ROS and if not connected, turn off arm
    #ifndef DEBUG
      if (!nh.connected()) {
        digitalWrite(LED, HIGH);
        axis1.stop();
        axis2.stop();
        setCanMotorSpeed(axis3, 0, 0);
        ros_connected = false;
      } else {
        digitalWrite(LED, LOW);
        ros_connected = true;
      }
    #endif

    if (!axis1.isRunning() && axis1_enabled) {
      axis1.disableOutputs();
      axis1_enabled = false;
    } else if (axis1.isRunning() && !axis1_enabled) {
      axis1.enableOutputs();
      axis1_enabled = true;
    }

    if (!axis2.isRunning() && axis2_enabled) {
      axis2.disableOutputs();
      axis2_enabled = false;
    } else if (axis2.isRunning() && !axis2_enabled) {
      axis2.enableOutputs();
      axis2_enabled = true;
    }


    #ifndef DEBUG

    #ifdef PID_TUNE
      pid_helper.y = arm_position.y;
      pid_helper.z = axis3.pid_speed;
      pidHelper.publish(&pid_helper);
    #endif


      float axis1_cur_rot = axis1.currentPosition() / axis1_steps_per_rad;
      joint_pos[0] = axis1_cur_rot; // tower axis
      joint_pos[1] = axis2.currentPosition() / axis2_steps_per_mm / 1000.00; //Z axis
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

  #endif

  switch (msg.buf[0]) {
      case 0x31: // encoder
        axis3.encoder = (returnEncoderRad(msg) / 4.00) - homed.axis3_offset;
        axis3.encoder = (axis3.encoder > 6) ? axis3.lower_limit : axis3.encoder;
        axis3.sent_get_encoder_command = false;
        //axis3.sent_position_command = false;
        //Serial.println(axis3.encoder);
        break;
      case 0x91:
        homed.axis3 = (msg.buf[1] == 2);
        axis3.sent_home_command = !(msg.buf[1] == 2);
        break;
      case 0xF5: //return from motor in absolute position command
        
        //getCanEncoderVal(axis3);
        break;
      }
}

void goHomeCan(can_motor &motor) {
  if (!motor.sent_home_command) {
  CAN_message_t msgSend;
  msgSend.len=2;
  msgSend.id= motor.id;
  msgSend.buf[0] = 0x91; //go home command
  msgSend.buf[1] = 0x92; //crc

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
  if (!motor.sent_get_encoder_command) {
    CAN_message_t msgSend;
    msgSend.len = 2;
    msgSend.id = motor.id;
    msgSend.buf[0] = 0x31; //encoder mode
    msgSend.buf[1] = msgSend.id + msgSend.buf[0]; //checksum
    motor.sent_get_encoder_command = true;
    can1.write(msgSend);
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


void home_axis() {
  if (!homed.axis3) {
    goHomeCan(axis3);
  } else if (!homed.axis2) {
    //If axis 2 isnt homed (z axis) then run axis down till it hits the end stop
    if (!digitalRead(limit_2_pin)) {
      axis2.setSpeed(homed.axis2_homing_speed * -axis2_steps_per_mm);
    } else { //once the endstop is triggered, set position 0 and tell the motor to move up 3mm. Wont move till all axis are done homing.
      axis2.stop();
      axis2.setCurrentPosition(homed.axis2_offset);
      //arm_position.z = 0.003;
      homed.axis2 = true;
    }
  } else if (!homed.axis1) {
    if (!digitalRead(limit_1_pin)) {
      axis1.setSpeed(homed.axis1_homing_speed * axis1_steps_per_rad); //homed.axis1_homing_speed * axis1_steps_per_rad
    } else { //once the endstop is triggered, set position 0 and tell the motor to move up 3mm. Wont move till all axis are done homing.
      axis1.setSpeed(0);
      axis1.stop();
      axis1.setCurrentPosition(homed.axis1_offset);
      //arm_position.x = 0.003;
     homed.axis1 = true;
     //runCanToPosition(axis3, arm_position.y, 500, 200);
    }
  } 

  if (homed.axis1 && homed.axis2 && homed.axis3) {
    has_homed = true;
    started_homing = false;
  }
  
}

