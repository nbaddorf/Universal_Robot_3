#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <Servo.h>
#include <std_msgs/Bool.h>
#include <ros/time.h>
//#include <tf/transform_datatypes.h>

char imu_base[] = "imu_base";

Servo soda;

bool servoState = false;

double lin_acceleration_covariance = 0.00001;
double ang_velocity_covariance = 0.00001;
double orien_covariance = 0.000001;

ros::NodeHandle nh;
//tf::Quaternion orientation;
sensor_msgs::Imu imu_msg;
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
 
MPU6050 mpu;
 
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]         
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3];         // [psi, theta, phi] 
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
 
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

ros::Publisher imu_pub("imu/data", &imu_msg);

void sodaDispencer(const std_msgs::Bool& state) {
  servoState = state.data;
}

ros::Subscriber<std_msgs::Bool> sodaSub("soda_dispencer" , sodaDispencer);
 
void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Wire.setWireTimeout(3000, true); //THIS MIGHT BE THE FIX!!
  Wire.clearWireTimeoutFlag();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
 
mpu.initialize();
//Serial.begin(115200);
devStatus = mpu.dmpInitialize();
 
    mpu.setXAccelOffset(-2987); //-1169
    mpu.setYAccelOffset(2259); //744
    mpu.setZAccelOffset(1301); //1620
    mpu.setXGyroOffset(43); //48
    mpu.setYGyroOffset(-39); //47
    mpu.setZGyroOffset(13); //-8
    
// make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
   // Serial.println(F(")"));
  }

  soda.attach(12);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(imu_pub);
  nh.subscribe(sodaSub);

  imu_msg.linear_acceleration_covariance[0] = 0.0000219368812815;//lin_acceleration_covariance;
  imu_msg.linear_acceleration_covariance[4] = 0.000110261624324;//lin_acceleration_covariance;
  imu_msg.linear_acceleration_covariance[8] = 10.42254612;//lin_acceleration_covariance;

  imu_msg.angular_velocity_covariance[0] = 0.0000136012649469;//ang_velocity_covariance;
  imu_msg.angular_velocity_covariance[4] = 0.0000263482257558;//ang_velocity_covariance;
  imu_msg.angular_velocity_covariance[8] = 0.0000527489061184;//ang_velocity_covariance;

  imu_msg.orientation_covariance[0] = 0.00187588539536;//orien_covariance;
  imu_msg.orientation_covariance[4] = 0.00146679937011;//orien_covariance;
  imu_msg.orientation_covariance[8] = 0.0687987915718;//orien_covariance;
}
 
 
void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
 
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);
 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    if (servoState) {
      soda.write(55);
    } else {
      soda.write(180);
    }
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    double wf = q.w/16384;
    double wx = q.x/16384;
    double wy = q.y/16384;
    double wz = q.z/16384;

    //tf::Quaternion orientation(xf, yf, zf, wf);

    double gx = ypr[0] * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
    double gy = ypr[1] * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
    double gz = ypr[2] * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
/*
    double ax = aaReal.x * (8.0 / 65536.0) * 9.81;
    double ay = aaReal.y * (8.0 / 65536.0) * 9.81;
    double az = aaReal.z * (8.0 / 65536.0) * 9.81;
*/
    double ax = aaReal.x * (8.0 / 65536.0) * 9.81;
    double ay = aaReal.y * (8.0 / 65536.0) * 9.81;
    double az = aaReal.z * (8.0 / 65536.0) * 9.81;

    //Serial.println(ax);

    //ros::time measurement_time = ros::Time::now()
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = imu_base;

    //quaternionTFToMsg(orientation, imu_msg.orientation);
    imu_msg.orientation.x = wx;
    imu_msg.orientation.y = wy;
    imu_msg.orientation.z = wz;
    imu_msg.orientation.w = wf;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_pub.publish(&imu_msg);

    nh.spinOnce();
    //imu_msg.
    /*
    Serial.print("Yaw: ");
    Serial.println(ypr[0] * 180/M_PI);
    Serial.print("Pitch: ");
    Serial.println(ypr[1] * 180/M_PI);
    Serial.print("Roll: ");
    Serial.println(ypr[2] * 180/M_PI);
    */
  }
}
