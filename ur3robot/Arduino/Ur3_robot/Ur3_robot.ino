//add all libraries to the sketch
#include <PID_v1.h>
#include <math.h>
//#include <Servo.h>
#include <PWMServo.h>

#include "AS5600.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>

//uncomment to enable debugging
//#define DEBUG

//uncomment to enable tf
//#define PUBLISH_TF

//Drive wheel math
float Wheel_Diameter = 96;                                      //in mm
float Wheel_Radious = Wheel_Diameter / 2;                       //in mm
float Wheel_Circumference = (2 * PI * (Wheel_Radious / 1000));  //Wheel Circumference in meters (0.30159289 m per rev)
float Wheel_Rev_Per_Meter = 1 / Wheel_Circumference;
// if wheel diam = 96 then wheel rev per meter = 3.31741 in calc agithnd arduino = 3.32

//560 encoder per motor rev
//gear reduction of 1:2
//30 tooth big gear, 15 tooth small gear.

float Encoder_Per_Wheel_Rev = 1120;  //in encoder counts I counted 1100 count per rev.
float Encoder_Counts_Per_Meter = Wheel_Rev_Per_Meter * Encoder_Per_Wheel_Rev; //3718 encoder per m
float Encoder_Counts_Per_MM = Encoder_Counts_Per_Meter / 1000;  //giving encoder counts per mm
//counts per mm == 3.718

double Odom_Width_Between_Wheels_X = 177;    //length between the two odom wheels in mm
double Odom_Width_Between_Wheels_Y = 254;  //length between the two odom wheels in mm
//double Odom_Width_Doubled = Odom_Width_Between_Wheels * 2;
double Odom_Wheel_Diameter = 72;                                           //in mm
double Odom_Wheel_Radious = Odom_Wheel_Diameter / 2;                       //in mm
double Odom_Wheel_Circumference = (2 * PI * (Odom_Wheel_Radious / 1000));  //Wheel Circumference in meters
double Odom_Wheel_Rev_Per_Meter = 1 / Odom_Wheel_Circumference;
//if diam = 72 then rev per meter = 4.42321

double Odom_Wheel_Encoder_Per_Wheel_Rev = 4095;  //in encoder counts
double Odom_Encoder_Half_Count = Odom_Wheel_Encoder_Per_Wheel_Rev / 2;
double Odom_Wheel_Encoder_Counts_Per_Meter = Odom_Wheel_Rev_Per_Meter * Odom_Wheel_Encoder_Per_Wheel_Rev;
double Odom_Wheel_Encoder_Counts_Per_MM = Odom_Wheel_Encoder_Counts_Per_Meter / 1000;  //giving encoder counts per mm
//couts per mm = 18.113

//drive motors
/*
float Distance_Between_Wheels = 378.1;                                                                                              //in mm                                                                                        //distance between left wheels and right wheels in mm
float Distance_Between_Wheels_Circumference = PI * 2 * Distance_Between_Wheels;     //2375.6724                                     //circumference in mm
float Distance_Between_Wheels_Half_Circumference_Rev = (Distance_Between_Wheels_Circumference / 2) / (Wheel_Circumference * 1000); //0.00314159  //Wheel Rev for half circumference
float Encoder_Counts_Per_Half_Circumference = Distance_Between_Wheels_Half_Circumference_Rev * Encoder_Per_Wheel_Rev;    //3.51858           //Outputs encoder counts per half circumference
float Encoder_Counts_Per_Radian = Encoder_Counts_Per_Half_Circumference / PI; // 1.12
*/

float Distance_Between_Wheels = 383;                                                                                              //in mm                                                                                        //distance between left wheels and right wheels in mm
float Distance_Between_Wheels_Half_Circumference = (PI * Distance_Between_Wheels) / 2;     //601.6 mm for 180 deg. = pi radians        
float Encoder_Counts_Per_Pi_Radian = Distance_Between_Wheels_Half_Circumference * Encoder_Counts_Per_MM; //2236.8 encoder counts
float Encoder_Counts_Per_Radian = Encoder_Counts_Per_Pi_Radian / PI; // 711.997 encoder counts per radian

double Front_Left_Encoder_Vel_Setpoint = 0;  //in count/10ms
double Front_Right_Encoder_Vel_Setpoint = 0;
double Back_Left_Encoder_Vel_Setpoint = 0;
double Back_Right_Encoder_Vel_Setpoint = 0;

bool low_battery_mode = false;

//create imu comlementery filter values
//float ST = 3.0/305.37254901960785;
//float tau = 0.1;
//float a = tau / (tau + ST);

const double velXAcceleration = 0.5; //mps^2 CHANGE THIS TO PROPER ACCELLERATION
const double velZAcceleration = 0.5; //mps^2



//drive base pid values
//float sampleTime = 0.5;
double kp = 0;   //0
double ki = 15;  //15
double kd = 0.001;   //0

float demandx;
float demandz;

//create ros msgs
ros::NodeHandle nh;

geometry_msgs::TransformStamped t;

nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

tf::TransformBroadcaster broadcaster;

sensor_msgs::BatteryState batt_state;
ros::Publisher batteryState("battery_state", &batt_state);

//define the motor control pins
const int motorFLspeedpin = 22;
const int motorBLspeedpin = 23;
const int motorFRspeedpin = 28;
const int motorBRspeedpin = 29;

//init the motor controllers
/*
Servo FL;
Servo BL;
Servo FR;
Servo BR;
*/

PWMServo FL;
PWMServo BL;
PWMServo FR;
PWMServo BR;

//********************** Setup AS5600 Encoder with multiplexer here *****************
const int odom_a0 = 30;  //These really shouldnt be here, these 3 pins just set the multiplexers address.
const int odom_a1 = 31;
const int odom_a2 = 32;

//init the odom encoders
AS5600 odom_front;
AS5600 odom_back;
AS5600 odom_left;
AS5600 odom_right;

//define pins for motor encoders
const int FLencoderA = 9;
const int FLencoderB = 8;
const int BLencoderA = 7;
const int BLencoderB = 6;
const int FRencoderA = 5;
const int FRencoderB = 4;
const int BRencoderA = 3;
const int BRencoderB = 2;

const int LED = 13;

const int voltageSensorPin = 21;
const int buzzerPin = 15;

char base_link[] = "base_link";  //was /base_link
char raw_odom[] = "raw_odom";
char odom[] = "odom";            //was /odom
//char laser[] = "laser"; //was /laser

// timers for the sub-main loop
unsigned long currentMillis;
unsigned long previousMillis = 0;  // set up timers
const float loopTime = 40; // 10

// timers for the sub-battery and buzzer loop
unsigned long previousBuzzerMillis = 0;  // set up timers
const float buzzerLoopTime = 2000;
bool buzzer_on = false;

// tf variables to be broadcast
double x = 0;
double y = 0;
double theta = 0;
double theta_old = 0;

double forward = 0;
double turn = 0;

double demandxAccel = 0;
double demandzAccel = 0;

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
/*
PID PID_FL(&FLpos_diff, &FLout, &Front_Left_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
PID PID_FR(&FRpos_diff, &FRout, &Front_Right_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
PID PID_BL(&BLpos_diff, &BLout, &Back_Left_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
PID PID_BR(&BRpos_diff, &BRout, &Back_Right_Encoder_Vel_Setpoint, kp, ki, kd, P_ON_M, DIRECT);
*/
PID PID_FL(&FLpos_diff, &FLout, &Front_Left_Encoder_Vel_Setpoint, kp, ki, kd, DIRECT);
PID PID_FR(&FRpos_diff, &FRout, &Front_Right_Encoder_Vel_Setpoint, kp, ki, kd, DIRECT);
PID PID_BL(&BLpos_diff, &BLout, &Back_Left_Encoder_Vel_Setpoint, kp, ki, kd, DIRECT);
PID PID_BR(&BRpos_diff, &BRout, &Back_Right_Encoder_Vel_Setpoint, kp, ki, kd, DIRECT);
// ** ROS callback & subscriber **

void velCallback(const geometry_msgs::Twist& vel) {
  demandx = constrain(vel.linear.x, -0.25, 0.25);
  demandz = constrain(vel.angular.z, -0.6, 0.6);
}

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
  #ifdef DEBUG
    Serial.begin(115200);
  #else
    // set baud rate to 115200
    nh.getHardware()->setBaud(115200);
    nh.initNode();  // init ROS

    nh.subscribe(pidPub);
    nh.subscribe(sub);
    //nh.subscribe(odomResetSub);

    nh.advertise(odom_pub);
    nh.advertise(batteryState);
    broadcaster.init(nh);  // set up broadcaster

     // Populate battery parameters.
    batt_state.design_capacity = 20000;      // mAh
    batt_state.power_supply_status = 2;     // discharging
    batt_state.power_supply_health = 0;     // unknown
    batt_state.power_supply_technology = 4; // LifePo4
    batt_state.present = 1;                 // battery present

    batt_state.location = "Main_Battery";        // unit location
    //batt_state.serial_number = "ABC_0001";  // unit serial number
    //batt_state.cell_voltage = new float[CELLS];
  #endif

  //set motor encoders to input pullup
  pinMode(FLencoderA, INPUT_PULLUP);
  pinMode(FLencoderB, INPUT_PULLUP);
  pinMode(FRencoderA, INPUT_PULLUP);
  pinMode(FRencoderB, INPUT_PULLUP);
  pinMode(BLencoderA, INPUT_PULLUP);
  pinMode(BLencoderB, INPUT_PULLUP);
  pinMode(BRencoderA, INPUT_PULLUP);
  pinMode(BRencoderB, INPUT_PULLUP);

  //attach interrupts to the motor encoders
  attachInterrupt(FRencoderA, FR0, RISING);
  attachInterrupt(FRencoderB, FR1, RISING);
  attachInterrupt(FLencoderA, FL0, RISING);
  attachInterrupt(FLencoderB, FL1, RISING);
  attachInterrupt(BRencoderA, BR0, RISING);
  attachInterrupt(BRencoderB, BR1, RISING);
  attachInterrupt(BLencoderA, BL0, RISING);
  attachInterrupt(BLencoderB, BL1, RISING);

  //set motor controllers to the correct pins
  FR.attach(motorFRspeedpin);
  BR.attach(motorBRspeedpin);
  FL.attach(motorFLspeedpin);
  BL.attach(motorBLspeedpin);

  pinMode(LED, OUTPUT);

  //set output for the odom multiplexer
  pinMode(odom_a0, OUTPUT);
  pinMode(odom_a1, OUTPUT);
  pinMode(odom_a2, OUTPUT);

  pinMode(buzzerPin, OUTPUT);

  /* Sets the TCA9548A multiplexer address to 0x70 */
  digitalWrite(odom_a0, LOW);
  digitalWrite(odom_a1, LOW);
  digitalWrite(odom_a2, LOW);
  delay(500);
  //start wire
  Wire.begin();
  delay(100);

  TCA9548A(0);                                 //front
  odom_front.begin();                          //  set direction pin.
  odom_front.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  TCA9548A(1);                                //back
  odom_back.begin();                          //  set direction pin.
  odom_back.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  TCA9548A(2);                                //left
  odom_left.begin();                          //  set direction pin.
  odom_left.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  TCA9548A(3);                                 //right
  odom_right.begin();                          //  set direction pin.
  odom_right.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  //set PID perameters
  PID_FL.SetOutputLimits(-50, 50);
  PID_FR.SetOutputLimits(-50, 50);
  PID_BL.SetOutputLimits(-50, 50);
  PID_BR.SetOutputLimits(-50, 50);
  PID_FL.SetMode(AUTOMATIC);
  PID_FR.SetMode(AUTOMATIC);
  PID_BL.SetMode(AUTOMATIC);
  PID_BR.SetMode(AUTOMATIC);
  /*
  PID_FL.SetSampleTime(sampleTime);
  PID_FR.SetSampleTime(sampleTime);
  PID_BL.SetSampleTime(sampleTime);
  PID_BR.SetSampleTime(sampleTime);
  */
  /*
  double p = 1;
  double i = 0;
  double d = 0.001;
  PID_FL.SetTunings(p, i, d);
  PID_FR.SetTunings(p, i, d);
  PID_BL.SetTunings(p, i, d);
  PID_BR.SetTunings(p, i, d);
  */
  // PID_LM.setPOnM(true);
  // PID_RM.setPOnM(true);

  previousMillis = millis();

  #ifdef DEBUG
    Serial.print("Wheel_Rev_Per_Meter:   ");
    Serial.println(Wheel_Rev_Per_Meter);
    //3.3157
    Serial.print("Encoder_Counts_Per_MM:    ");
    //6.219375
    Serial.println(Encoder_Counts_Per_Radian);
    Serial.print("Odom_Wheel_Encoder_Counts_Per_radian:    ");
    Serial.println(Odom_Wheel_Encoder_Counts_Per_MM);
    delay(5000);
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  #ifndef DEBUG
  nh.spinOnce();  // make sure we listen for ROS messages and activate the callback if there is one
  #endif

  currentMillis = millis();

  if (currentMillis - previousBuzzerMillis >= buzzerLoopTime && low_battery_mode) {  // run a loop every 2 seconds
    previousBuzzerMillis = currentMillis;
   
   if (buzzer_on) {
     noTone(buzzerPin);
     buzzer_on = false;
   } else {
     tone(buzzerPin, 1000);
     buzzer_on = true;
   }
  }

  if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms
    previousMillis = currentMillis;

    //Check Battery Level
    //1024 bit resolution, 3.3 = 16.5v
    const float R1 = 30000; // 30kohm
    const float R2 = 7500; //7.5 kohm
    int voltageValue = analogRead(voltageSensorPin);
    double voltage = ((voltageValue * 3.25) / 1024) / (R2/(R1+R2));
    //float vin = vout / (R2/(R1+R2));
    #ifndef DEBUG
      if (voltage <= 12.8) {
        batt_state.power_supply_health = 3; // Dead
        if (!low_battery_mode) {
          previousBuzzerMillis = currentMillis;
        }
        low_battery_mode = true;
      } else if (voltage >= 14.0) {
        batt_state.power_supply_health = 3; //overvoltage
      } else {
        batt_state.power_supply_health = 1; // good
        low_battery_mode = false;
      }

      batt_state.voltage = voltage;
      batteryState.publish( &batt_state );
    #endif

    float modifier_lin = 1.00;  // scaling factor because the wheels are squashy / there is wheel slip etc.
    float modifier_ang = 1.00;  // scaling factor because the wheels are squashy / there is wheel slip etc.
    

    //ACCELERATION code here
    const double velXAccelerationPerLoop = velXAcceleration / (1000/loopTime);
    const double velZAccelerationPerLoop = velZAcceleration / (1000/loopTime);

    //x acceleration code
    if (demandxAccel != demandx) {
      if (demandx > demandxAccel) {
        demandxAccel += velXAccelerationPerLoop;
        demandxAccel = constrain(demandxAccel, -1.0, demandx);
      } else if (demandx < demandxAccel) {
        demandxAccel -= velXAccelerationPerLoop;
        demandxAccel = constrain(demandxAccel, demandx, 1.0);
      } 
    }

    //z acceleration code
    if (demandzAccel != demandz) {
      if (demandz > demandzAccel) {
        demandzAccel += velZAccelerationPerLoop;
        demandzAccel = constrain(demandzAccel, -1.0, demandz);
      } else if (demandz < demandzAccel) {
        demandzAccel -= velZAccelerationPerLoop;
        demandzAccel = constrain(demandzAccel, demandz, 1.0);
      } 
    }

    //forward = demandxAccel * (Encoder_Counts_Per_Meter * modifier_lin);
    //turn = demandzAccel * (Encoder_Counts_Per_Radian * modifier_ang); // tells us how many encoder counts per second to turn
    
    forward = 0 * (Encoder_Counts_Per_Meter * modifier_lin);
    turn = 0.2 * (Encoder_Counts_Per_Radian * modifier_ang); //142.39

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
    #ifndef DEBUG
      if (!nh.connected()) {
        digitalWrite(LED, HIGH);
        FLout = 0;
        FRout = 0;
        BLout = 0;
        BRout = 0;
      } else {
        digitalWrite(LED, LOW);
      }
    #endif
   
    //Set motors to run at speeds
    setMotorSpeed(FLout, BLout, FRout, BRout);

    //Calculate chainge in encoder rotations
    //*****Should this be moved up before the pid computes? *****
    FLpos_diff = FLpos - FLpos_old;
    FRpos_diff = FRpos - FRpos_old;
    BLpos_diff = BLpos - BLpos_old;
    BRpos_diff = BRpos - BRpos_old;

    #ifdef DEBUG
      Serial.print("encoder: ");
      Serial.println(FRpos); // 2.84
      Serial.print("speed: ");
      Serial.println(FRout);
      Serial.print("speed In: ");
      Serial.println(Front_Right_Encoder_Vel_Setpoint);
      Serial.print("diff: ");
      Serial.println(FRpos_diff);
    #endif
    
    FLpos_old = FLpos;
    FRpos_old = FRpos;
    BLpos_old = BLpos;
    BRpos_old = BRpos;

    //getOdomValues(&odom_f, &odom_b, odom_l, &odom_r);
    getOdomValues(&FOdomPos, &BOdomPos, &LOdomPos, &ROdomPos);
    
    LOdomPos = map(LOdomPos, 0, Odom_Wheel_Encoder_Per_Wheel_Rev, Odom_Wheel_Encoder_Per_Wheel_Rev, 0);
    BOdomPos = map(BOdomPos, 0, Odom_Wheel_Encoder_Per_Wheel_Rev, Odom_Wheel_Encoder_Per_Wheel_Rev, 0);

    FOdomPos_diff = check_odom_rollover(&FOdomPos, &FOdomPos_old);
    BOdomPos_diff = check_odom_rollover(&BOdomPos, &BOdomPos_old);
    LOdomPos_diff = check_odom_rollover(&LOdomPos, &LOdomPos_old);
    ROdomPos_diff = check_odom_rollover(&ROdomPos, &ROdomPos_old);
    
    /*
    #ifdef DEBUG
      Serial.print("front odom:    ");
      Serial.println(FOdomPos_diff);
      Serial.print("back odom:    ");
      Serial.println(BOdomPos_diff);
      Serial.print("left odom:    ");
      Serial.println(LOdomPos_diff);
      Serial.print("right odom:    ");
      Serial.println(ROdomPos_diff);
    #endif
*/

  FOdomPos_old = FOdomPos;
  BOdomPos_old = BOdomPos;
  LOdomPos_old = LOdomPos;
  ROdomPos_old = ROdomPos;

  float Left_mm_diff = (LOdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;
  float Right_mm_diff = (ROdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;
  float Back_mm_diff = (BOdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;
  float Front_mm_diff = (FOdomPos_diff) / Odom_Wheel_Encoder_Counts_Per_MM;

  /*
  #ifdef DEBUG
    Serial.print("front mm:    ");
    Serial.println(Front_mm_diff);
    Serial.print("back mm:    ");
    Serial.println(Back_mm_diff);
    Serial.print("left mm:    ");
    Serial.println(Left_mm_diff);
    Serial.print("right mm:    ");
    Serial.println(Right_mm_diff);
  #endif
  */

  // calc distance travelled based on average of both wheels
  pos_x_average_mm_diff = (Left_mm_diff + Right_mm_diff) / 2;  // difference in each cycle
  pos_x_total_mm += pos_x_average_mm_diff;                     // calc total running total distance
  // calc distance travelled based on average of both wheels
  pos_y_average_mm_diff = (Back_mm_diff + Front_mm_diff) / 2;  // difference in each cycle
  pos_y_total_mm += pos_y_average_mm_diff;                       // calc total running total distance

  #ifdef DEBUG
    //Serial.print("x distance:    ");
    //Serial.println(pos_x_total_mm);
  #endif

  // calc angle or rotation to broadcast with tf
  float phi_x = ((Right_mm_diff - Left_mm_diff) / Odom_Width_Between_Wheels_X);
  float phi_y = ((Front_mm_diff - Back_mm_diff) / Odom_Width_Between_Wheels_Y);

  theta += (phi_x + phi_y) / 2;
  //theta += (phi_x);

  if (theta >= TWO_PI) {
    theta -= TWO_PI;
  }
  if (theta <= (-TWO_PI)) {
    theta += TWO_PI;
  }

  #ifdef DEBUG
   // Serial.print("rotation:    ");
    //Serial.println(theta);
  #endif

  // calc x and y to broadcast with tf
  //maybe make 2 vars one for sin and other for cos so that the arduino doesnt have to do the math for sin and cos twice.
  y += (pos_x_average_mm_diff * sin(theta)) + (pos_y_average_mm_diff * cos(theta));
  x += (pos_x_average_mm_diff * cos(theta)) + (pos_y_average_mm_diff * sin(theta));

  #ifdef DEBUG
  //Serial.println(((Left_mm_diff + Right_mm_diff) / 2) / 10);
    //Serial.print("x:    ");
    //Serial.println(x);
    //Serial.print("y:    ");
    //Serial.println(y);
  #endif

  #ifdef PUBLISH_TF         
            // *** broadcast odom->base_link transform with tf ***
            geometry_msgs::TransformStamped t;
                      
            t.header.frame_id = odom;
           // t.child_frame_id = base_link;
           t.child_frame_id = raw_odom;
            
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
  #endif


  // *** broadcast odom message ***
  #ifndef DEBUG
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x / 1000;
    odom_msg.pose.pose.position.y = y / 1000;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
    //odom_msg.pose.pose.orientation.x = angleXtotal;

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = ((Left_mm_diff + Right_mm_diff) / 2) / loopTime;// / 10 instead of loopTime  // forward linear velocity
    odom_msg.twist.twist.linear.y = ((Front_mm_diff + Back_mm_diff) / 2) / loopTime;  // sideways linear velocity                                        // robot does not move sideways
    //odom_msg.twist.twist.angular.z = (((Right_mm_diff - Left_mm_diff) / Odom_Width_Between_Wheels_X) + ((Back_mm_diff - Front_mm_diff) / Odom_Width_Between_Wheels_Y) / 2); * 100;
    odom_msg.twist.twist.angular.z = (theta - theta_old) * (1000/loopTime);
    //odom_msg.twist.covariance[0] = 0.00005;  //x vel
    //odom_msg.twist.covariance[20] = 0.001;   // rotation around z vel
                                            // odom_msg.twist.twist.angular.x = angleXdif;// anglular velocity
    //nh.spinOnce();
    odom_pub.publish(&odom_msg);

    theta_old = theta;

    nh.spinOnce();
  #endif

}  // end of 10ms loop
}

void TCA9548A(uint8_t bus) {
  if (bus == 0) {  // front
    bus = 7;
  } else if (bus == 1) {  // back
    bus = 6;
  } else if (bus == 2) {  // left
    bus = 5;
  } else if (bus == 3) {  // right
    bus = 4;
  }
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

//void getOdomValues(uint16_t* f, uint16_t* b, uint16_t* l, uint16_t* r) {
void getOdomValues(int* _f, int* _b, int* _l, int* _r) {
  TCA9548A(0);
  //Serial.println()
  *_f = odom_front.rawAngle();
  TCA9548A(1);
  *_b = odom_back.rawAngle();
  TCA9548A(2);
  *_l = odom_left.rawAngle();
  TCA9548A(3);
  *_r = odom_right.rawAngle();
}

int check_odom_rollover(int* valIn, int* val_old) {
  if (*valIn == *val_old) {
      return 0;
  } else if (abs(*valIn - *val_old) >= Odom_Encoder_Half_Count) {
    if (*valIn < *val_old) {
      return (Odom_Wheel_Encoder_Per_Wheel_Rev - *val_old) + *valIn; // + 1;
    } else if (*valIn > *val_old) {
      return -(Odom_Wheel_Encoder_Per_Wheel_Rev - *valIn) + *val_old; // + 1;
    }
  } else {
    return *valIn - *val_old;
  }
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
