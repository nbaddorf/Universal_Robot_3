 //add all libraries to the sketch
#include <PID_v1.h>
#include <math.h>
#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

float Wheel_Diameter = 96;                                      //in mm
float Wheel_Radious = Wheel_Diameter / 2;                       //in mm
float Wheel_Circumference = (2 * PI * (Wheel_Radious / 1000));  //Wheel Circumference in meters
float Wheel_Rev_Per_Meter = 1 / Wheel_Circumference;
// if wheel diam = 96 then wheel rev per meter = 3.31741 in calc agithnd arduino = 3.32

// 625 encoder per motor rev
//gear reduction of 1:3

float Encoder_Per_Wheel_Rev = 1875;  //in encoder counts
float Encoder_Counts_Per_Meter = Wheel_Rev_Per_Meter * Encoder_Per_Wheel_Rev;
float Encoder_Counts_Per_MM = Encoder_Counts_Per_Meter / 1000;  //giving encoder counts per mm
//counts per mm == 4.97612

double Odom_Width_Between_Wheels_X = 175;  //length between the two odom wheels in mm
double Odom_Width_Between_Wheels_Y = 251.2;  //length between the two odom wheels in mm
//double Odom_Width_Doubled = Odom_Width_Between_Wheels * 2;
double Odom_Wheel_Diameter = 72;                                           //in mm
double Odom_Wheel_Radious = Odom_Wheel_Diameter / 2;                       //in mm
double Odom_Wheel_Circumference = (2 * PI * (Odom_Wheel_Radious / 1000));  //Wheel Circumference in meters
double Odom_Wheel_Rev_Per_Meter = 1 / Odom_Wheel_Circumference;
//if diam = 72 then rev per meter = 4.42321

// ******************************UPDATE THIS **************************
double Odom_Wheel_Encoder_Per_Wheel_Rev = 720;  //in encoder counts
double Odom_Wheel_Encoder_Counts_Per_Meter = Odom_Wheel_Rev_Per_Meter * Odom_Wheel_Encoder_Per_Wheel_Rev;
double Odom_Wheel_Encoder_Counts_Per_MM = Odom_Wheel_Encoder_Counts_Per_Meter / 1000;  //giving encoder counts per mm
//couts per mm = 1.59236

float Distance_Between_Wheels = 378.1;  //in mm                                                                                        //distance between left wheels and right wheels in mm
float Distance_Between_Wheels_Circumference = PI * 2 * Distance_Between_Wheels;                                                     //circumference in mm
float Distance_Between_Wheels_Half_Circumference_Rev = (Distance_Between_Wheels_Circumference / 2) / (Wheel_Circumference * 1000);  //Wheel Rev for half circumference
float Encoder_Counts_Per_Half_Circumference = Distance_Between_Wheels_Half_Circumference_Rev * Encoder_Per_Wheel_Rev;               //Outputs encoder counts per half circumference
float Encoder_Counts_Per_Radian = Encoder_Counts_Per_Half_Circumference / PI;

double Front_Left_Encoder_Vel_Setpoint = 0;  //in count/10ms
double Front_Right_Encoder_Vel_Setpoint = 0;
double Back_Left_Encoder_Vel_Setpoint = 0;
double Back_Right_Encoder_Vel_Setpoint = 0;

//create imu comlementery filter values
//float ST = 3.0/305.37254901960785;
//float tau = 0.1;
//float a = tau / (tau + ST);



//drive base pid values
float sampleTime = 0.5;
double kp = 0;   //0.4
double ki = 15;  //4.2
double kd = 0;   //0

float demandx;
float demandz;

//create ros msgs
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

//set up which pins controll what
const int motorFLspeedpin = 22;
const int motorBLspeedpin = 23;
const int motorFRspeedpin = 28;
const int motorBRspeedpin = 29;

Servo FL;
Servo BL;
Servo FR;
Servo BR;

//********************** Setup AS5600 Encoder with multiplexer here *****************


const int FLencoderA = 9;
const int FLencoderB = 8;
const int BLencoderA = 7;
const int BLencoderB = 6;
const int FRencoderA = 5;
const int FRencoderB = 4;
const int BRencoderA = 3;
const int BRencoderB = 2;

/*
const int LeftOdomEncoderA = 11;
const int LeftOdomEncoderB = 10;
const int RightOdomEncoderA = 12;
const int RightOdomEncoderB = 0;
*/

const int LED = 13;

char base_link[] = "base_link";  //was /base_link
char odom[] = "odom";            //was /odom
//char laser[] = "laser"; //was /laser

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0;  // set up timers
float loopTime = 10;

// tf variables to be broadcast
double x = 0;
double y = 0;
double theta = 0;

double forward = 0;
double turn = 0;

char info[50];

double FLout = 0;
double FRout = 0;
double BLout = 0;
double BRout = 0;

int BLpos = 0;
int FLpos = 0;
int BRpos = 0;
int FRpos = 0;

int LOdomPos = 0;
int ROdomPos = 0;
int BOdomPos = 0;
int FOdomPos = 0;

double BLpos_diff = 0;  //in encoder counts per 10ms
double FLpos_diff = 0;
double BRpos_diff = 0;
double FRpos_diff = 0;

double LOdomPos_diff = 0;
double ROdomPos_diff = 0;
double BOdomPos_diff = 0;
double FOdomPos_diff = 0;

double FLpos_mm_diff;
double FRpos_mm_diff;
double BLpos_mm_diff;
double BRpos_mm_diff;

double LOdomPos_mm_diff;
double ROdomPos_mm_diff;
double BOdomPos_mm_diff;
double FOdomPos_mm_diff;

long BLpos_old = 0;
long FLpos_old = 0;
long BRpos_old = 0;
long FRpos_old = 0;

long LOdomPos_old = 0;
long ROdomPos_old = 0;
long BOdomPos_old = 0;
long FOdomPos_old = 0;

float pos_x_average_mm_diff;
float pos_x_total_mm;
float pos_y_average_mm_diff;
float pos_y_total_mm;

//bool odomResetState = false;

//create pid for drivebase
PID PID_FL(&FLpos_diff, &FLout, &Front_Left_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
PID PID_FR(&FRpos_diff, &FRout, &Front_Right_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
PID PID_BL(&BLpos_diff, &BLout, &Back_Left_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
PID PID_BR(&BRpos_diff, &BRout, &Back_Right_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);

// ** ROS callback & subscriber **

void velCallback(const geometry_msgs::Twist& vel) {
  demandx = constrain(vel.linear.x, -0.2, 0.2);
  demandz = constrain(vel.angular.z, -0.4, 0.4);
}
/*
void resetOdom(const std_msgs::Bool& state) {
  //if (state && (odomResetState == false)) {
  LOdomPos = 0;
  ROdomPos = 0;
  LOdomPos_diff = 0;
  ROdomPos_diff = 0;
  LOdomPos_mm_diff = 0;
  ROdomPos_mm_diff = 0;
  LOdomPos_old = 0;
  ROdomPos_old = 0;
  pos_average_mm_diff = 0;
  pos_total_mm = 0;
  x = 0;
  y = 0;
  theta = 0;
  // odomResetState = true;
  //} else if (state == false) {
  // odomResetState = false;
  //}
}
*/
void pidSet(const geometry_msgs::Vector3& vel) {
  int p = vel.x;
  int i = vel.y;
  int d = vel.z;
  PID_FL.SetTunings(p, i, d);
  PID_FR.SetTunings(p, i, d);
  PID_BL.SetTunings(p, i, d);
  PID_BR.SetTunings(p, i, d);
}


ros::Subscriber<geometry_msgs::Vector3> pidPub("set_pid", pidSet);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);
//ros::Subscriber<std_msgs::Bool> odomResetSub("reset_odom", resetOdom);

void setup() {
  // set baud rate to 115200
  nh.getHardware()->setBaud(115200);
  nh.initNode();  // init ROS

  nh.subscribe(pidPub);
  nh.subscribe(sub);
  //nh.subscribe(odomResetSub);

  nh.advertise(odom_pub);
  broadcaster.init(nh);  // set up broadcaster


  pinMode(FLencoderA, INPUT_PULLUP);
  pinMode(FLencoderB, INPUT_PULLUP);
  pinMode(FRencoderA, INPUT_PULLUP);
  pinMode(FRencoderB, INPUT_PULLUP);
  pinMode(BLencoderA, INPUT_PULLUP);
  pinMode(BLencoderB, INPUT_PULLUP);
  pinMode(BRencoderA, INPUT_PULLUP);
  pinMode(BRencoderB, INPUT_PULLUP);
  /*
  pinMode(LeftOdomEncoderA, INPUT_PULLUP); 
  pinMode(LeftOdomEncoderB, INPUT_PULLUP);
  pinMode(RightOdomEncoderA, INPUT_PULLUP);
  pinMode(RightOdomEncoderB, INPUT_PULLUP);
*/
  attachInterrupt(FRencoderA, FR0, RISING);
  attachInterrupt(FRencoderB, FR1, RISING);
  attachInterrupt(FLencoderA, FL0, RISING);
  attachInterrupt(FLencoderB, FL1, RISING);
  attachInterrupt(BRencoderA, BR0, RISING);
  attachInterrupt(BRencoderB, BR1, RISING);
  attachInterrupt(BLencoderA, BL0, RISING);
  attachInterrupt(BLencoderB, BL1, RISING);
  /*
  attachInterrupt(LeftOdomEncoderA, LO0, RISING);
  attachInterrupt(LeftOdomEncoderB, LO1, RISING);
  attachInterrupt(RightOdomEncoderA, RO0, RISING);
  attachInterrupt(RightOdomEncoderB, RO1, RISING);
*/
  FR.attach(motorFRspeedpin);
  BR.attach(motorBRspeedpin);
  FL.attach(motorFLspeedpin);
  BL.attach(motorBLspeedpin);

  pinMode(LED, OUTPUT);

  PID_FL.SetOutputLimits(-50, 50);
  PID_FR.SetOutputLimits(-50, 50);
  PID_BL.SetOutputLimits(-50, 50);
  PID_BR.SetOutputLimits(-50, 50);
  PID_FL.SetMode(AUTOMATIC);
  PID_FR.SetMode(AUTOMATIC);
  PID_BL.SetMode(AUTOMATIC);
  PID_BR.SetMode(AUTOMATIC);
  PID_FL.SetSampleTime(sampleTime);
  PID_FR.SetSampleTime(sampleTime);
  PID_BL.SetSampleTime(sampleTime);
  PID_BR.SetSampleTime(sampleTime);
  // PID_LM.setPOnM(true);
  // PID_RM.setPOnM(true);

  //Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();  // make sure we listen for ROS messages and activate the callback if there is one

  currentMillis = millis();
  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    float modifier_lin = 1.00;  // scaling factor because the wheels are squashy / there is wheel slip etc.
    float modifier_ang = 1.00;  // scaling factor because the wheels are squashy / there is wheel slip etc.

    forward = demandx * (Encoder_Counts_Per_Meter * modifier_lin);
    turn = demandz * (Encoder_Counts_Per_Radian * modifier_ang);

    float Left_Motor_Speed = forward - turn;  //in encoder counts per second
    float Right_Motor_Speed = forward + turn;

    Left_Motor_Speed = Left_Motor_Speed / (1000 / loopTime);  //loop runs every 10ms so divide final0/(1000/loopTime) to get counts per 10ms
    Right_Motor_Speed = Right_Motor_Speed / (1000 / loopTime);

    Front_Left_Encoder_Vel_Setpoint = Left_Motor_Speed;
    Front_Right_Encoder_Vel_Setpoint = Right_Motor_Speed;
    Back_Left_Encoder_Vel_Setpoint = Left_Motor_Speed;
    Back_Right_Encoder_Vel_Setpoint = Right_Motor_Speed;

    //Compute PID speeds
    PID_FL.Compute();
    PID_FR.Compute();
    PID_BL.Compute();
    PID_BR.Compute();

    //Reset PID if speed is 0
    if ((Front_Left_Encoder_Vel_Setpoint * 1000 == 0) && (abs(FLout) <= 5)) {
      FLout = 0;
      PID_FL.SetMode(MANUAL);
    } else {
      PID_FL.SetMode(AUTOMATIC);
    }
    if ((Front_Right_Encoder_Vel_Setpoint * 1000 == 0) && (abs(FRout) <= 5)) {
      FRout = 0;
      PID_FR.SetMode(MANUAL);
    } else {
      PID_FR.SetMode(AUTOMATIC);
    }
    if ((Back_Left_Encoder_Vel_Setpoint * 1000 == 0) && (abs(BLout) <= 5)) {
      BLout = 0;
      PID_BL.SetMode(MANUAL);
    } else {
      PID_BL.SetMode(AUTOMATIC);
    }
    if ((Back_Right_Encoder_Vel_Setpoint * 1000 == 0) && (abs(BRout) <= 5)) {
      BRout = 0;
      PID_BR.SetMode(MANUAL);
    } else {
      PID_BR.SetMode(AUTOMATIC);
    }

    //Check if robot is connected to ROS and if not connected, turn off wheels
    if (!nh.connected()) {
      digitalWrite(LED, HIGH);
      FLout = 0;
      FRout = 0;
      BLout = 0;
      BRout = 0;
    } else {
      digitalWrite(LED, LOW);
    }

    //Set motors to run at speeds
    setMotorSpeed(FLout, BLout, FRout, BRout);

    //Calculate chainge in encoder rotations
    //*****Should this be moved up before the pid computes? *****
    FLpos_diff = FLpos - FLpos_old;
    FRpos_diff = FRpos - FRpos_old;
    BLpos_diff = BLpos - BLpos_old;
    BRpos_diff = BRpos - BRpos_old;

    LOdomPos_diff = LOdomPos - LOdomPos_old;
    ROdomPos_diff = ROdomPos - ROdomPos_old;
    //BOdomPos_diff = ...
    //FOdomPos_diff = ...
    //Serial.println(ROdomPos);

    FLpos_old = FLpos;
    FRpos_old = FRpos;
    BLpos_old = BLpos;
    BRpos_old = BRpos;

    LOdomPos_old = LOdomPos;
    ROdomPos_old = ROdomPos;
    //BOdomPos_old = ...
    //FOdomPos_old = ...

    float Left_mm_diff = (LOdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;
    float Right_mm_diff = (ROdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;
    //float Back_mm_diff = (BOdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;
    //float Front_mm_diff = (FOdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;

    //Serial.println(Right_mm_diff);

    // calc distance travelled based on average of both wheels
    pos_x_average_mm_diff = (Left_mm_diff + Right_mm_diff) / 2;  // difference in each cycle
    pos_x_total_mm += pos_x_average_mm_diff;                       // calc total running total distance
    // calc distance travelled based on average of both wheels
    //pos_y_average_mm_diff = (Back_mm_diff + Front_mm_diff) / 2;  // difference in each cycle
    //pos_y_total_mm += pos_y_average_mm_diff;                       // calc total running total distance
    

    // calc angle or rotation to broadcast with tf
    float phi_x = ((Right_mm_diff - Left_mm_diff) / Odom_Width_Between_Wheels_X);
    //float phi_y = ((Front_mm_diff - Back_mm_diff) / Odom_Width_Between_Wheels_Y);

    theta += (phi_x + phi_y) / 2;

    if (theta >= TWO_PI) {
      theta -= TWO_PI;
    }
    if (theta <= (-TWO_PI)) {
      theta += TWO_PI;
    }

    // calc x and y to broadcast with tf
    //maybe make 2 vars one for sin and other for cos so that the arduino doesnt have to do the math for sin and cos twice.
    y += (pos_x_average_mm_diff * sin(theta)) + (pos_y_average_mm_diff * cos(theta));
    x += (pos_x_average_mm_diff * cos(theta)) + (pos_y_average_mm_diff * sin(theta));

    /*          
            // *** broadcast odom->base_link transform with tf ***
            geometry_msgs::TransformStamped t;
                      
            t.header.frame_id = odom;
            t.child_frame_id = base_link;
            
            t.transform.translation.x = x/1000;   // convert to metres
            t.transform.translation.y = y/1000;
            t.transform.translation.z = 0;

            float yaw1 = theta;
            float pitch1=0;
            float roll1 = 0;
            //converts euler angle to quaternion
            float baseqx = sin(roll1/2) * cos(pitch1/2) * cos(yaw1/2) - cos(roll1/2) * sin(pitch1/2) * sin(yaw1/2);
            float baseqy = cos(roll1/2) * sin(pitch1/2) * cos(yaw1/2) + sin(roll1/2) * cos(pitch1/2) * sin(yaw1/2);
            float baseqz = cos(roll1/2) * cos(pitch1/2) * sin(yaw1/2) - sin(roll1/2) * sin(pitch1/2) * cos(yaw1/2);
            float baseqw = cos(roll1/2) * cos(pitch1/2) * cos(yaw1/2) + sin(roll1/2) * sin(pitch1/2) * sin(yaw1/2);
            //adds the quaternion to the ros message
            t.transform.rotation.x = baseqx;
            t.transform.rotation.y = baseqy;
            t.transform.rotation.z = baseqz;
            t.transform.rotation.w = baseqw;
            //t.transform.rotation = tf::createQuaternionFromYaw(theta);
            t.header.stamp = nh.now();      
            broadcaster.sendTransform(t);
*/
    /*
            //Send transform for the lidar
            geometry_msgs::TransformStamped li;
            li.header.frame_id = base_link;
            li.child_frame_id = laser;
            li.transform.translation.x = 0; 
            li.transform.translation.y = 0;
            li.transform.translation.z = 0.2159;
            
            float LaserPitch = 0;
            float LaserRoll = 0;
            float LaserYaw = PI/2;
            //converts euler angle to quaternion
            float qx = sin(LaserRoll/2) * cos(LaserPitch/2) * cos(LaserYaw/2) - cos(LaserRoll/2) * sin(LaserPitch/2) * sin(LaserYaw/2);
            float qy = cos(LaserRoll/2) * sin(LaserPitch/2) * cos(LaserYaw/2) + sin(LaserRoll/2) * cos(LaserPitch/2) * sin(LaserYaw/2);
            float qz = cos(LaserRoll/2) * cos(LaserPitch/2) * sin(LaserYaw/2) - sin(LaserRoll/2) * sin(LaserPitch/2) * cos(LaserYaw/2);
            float qw = cos(LaserRoll/2) * cos(LaserPitch/2) * cos(LaserYaw/2) + sin(LaserRoll/2) * sin(LaserPitch/2) * sin(LaserYaw/2);
            //adds the quaternion to the ros message
            li.transform.rotation.x = qx;
            li.transform.rotation.y = qy;
            li.transform.rotation.z = qz;
            li.transform.rotation.w = qw;
            //ros publishes the tf for the lidar
            li.header.stamp = nh.now();
            broadcaster.sendTransform(li);
*/

    // *** broadcast odom message ***
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x / 1000;
    odom_msg.pose.pose.position.y = y / 1000;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
    //odom_msg.pose.pose.orientation.x = angleXtotal;

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = ((Left_mm_diff + Right_mm_diff) / 2) / 10;  // forward linear velocity
    odom_msg.twist.twist.linear.y = ((Front_mm_diff + Back_mm_diff) / 2) / 10;  // sideways linear velocity                                        // robot does not move sideways
    odom_msg.twist.twist.angular.z = (((Right_mm_diff - Left_mm_diff) / Odom_Width_Between_Wheels_X) + ((Back_mm_diff - Front_mm_diff) / Odom_Width_Between_Wheels_Y) / 2) * 100;
    odom_msg.twist.covariance[0] = 0.00005;  //x vel
    odom_msg.twist.covariance[20] = 0.001;   // rotation around z vel
                                             // odom_msg.twist.twist.angular.x = angleXdif;// anglular velocity
    //nh.spinOnce();
    odom_pub.publish(&odom_msg);


    nh.spinOnce();

  }  // end of 10ms loop
}

void BR0() {
  if (digitalRead(BRencoderB) == LOW) {
    BRpos++;
  } else {
    BRpos--;
  }
}

void BR1() {
  if (digitalRead(BRencoderA) == LOW) {
    BRpos--;
  } else {
    BRpos++;
  }
}

void BL0() {
  if (digitalRead(BLencoderB) == LOW) {
    BLpos--;
  } else {
    BLpos++;
  }
}

void BL1() {
  if (digitalRead(BLencoderA) == LOW) {
    BLpos++;
  } else {
    BLpos--;
  }
}

void FR0() {
  if (digitalRead(FRencoderB) == LOW) {
    FRpos++;
  } else {
    FRpos--;
  }
}

void FR1() {
  if (digitalRead(FRencoderA) == LOW) {
    FRpos--;
  } else {
    FRpos++;
  }
}

void FL0() {
  if (digitalRead(FLencoderB) == LOW) {
    FLpos--;
  } else {
    FLpos++;
  }
}

void FL1() {
  if (digitalRead(FLencoderA) == LOW) {
    FLpos++;
  } else {
    FLpos--;
  }
}

/*
void LO0() {
  if (digitalRead(LeftOdomEncoderB)==LOW) {
  LOdomPos++;
  } else {
  LOdomPos--;
  }
  }

void LO1() {
  if (digitalRead(LeftOdomEncoderA)==LOW) {
  LOdomPos--;
  } else {
  LOdomPos++;
  }
  }

void RO0() {
  if (digitalRead(RightOdomEncoderB)==LOW) {
  ROdomPos++;
  } else {
  ROdomPos--;
  }
  }

void RO1() {
  if (digitalRead(RightOdomEncoderA)==LOW) {
  ROdomPos--;
  } else {
  ROdomPos++;
  }
  }
*/

void setMotorSpeed(float fl, float bl, float fr, float br) {
  fl = map(fl, -100, 100, 180, 0);
  fr = map(fr, -100, 100, 180, 0);
  bl = map(bl, -100, 100, 180, 0);
  br = map(br, -100, 100, 180, 0);
  FL.write(fl);
  FR.write(fr);
  BL.write(bl);
  BR.write(br);
}
