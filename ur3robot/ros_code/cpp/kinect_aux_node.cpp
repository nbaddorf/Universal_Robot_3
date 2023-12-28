#include <libusb-1.0/libusb.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <cmath>

//########################
//The reson I am using the kinect aux code in my program is cause I fixed a bug switching a uint16 to int16 allowing the kinect to tilt downward



// VID and PID for Kinect and motor/acc/leds
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
// Constants for accelerometers
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.
// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
//#define MAX_TILT_ANGLE 18. //31
//#define MIN_TILT_ANGLE (-44.) //31

double old_tilt_angle = 0;
bool has_homed = false;

int MAX_TILT_ANGLE = 31; //18
int MIN_TILT_ANGLE = (-31.); //-44

int init_tilt_angle = 0;

bool pub_tf = false;
bool pub_joint_state = false;
bool use_imu_for_angle = false;

std::string kinect_pivot_tf_name;
std::string kinect_camera_tf_name;
std::string kinect_pivot_joint_name; //name of joint that controlls the kinect tilt in urdf

ros::Publisher pub_imu;
ros::Publisher pub_tilt_angle;
ros::Publisher pub_tilt_status;
ros::Publisher joint_pub;

ros::Subscriber sub_tilt_angle;
ros::Subscriber sub_led_option;
 
libusb_device_handle *dev(0);

static double d2r(double d) {
  static const auto PI = std::acos(-1);
  return (d / 180.0) * PI;
}

double cosd(double x /* degrees */) {
  if (!isfinite(x)) {
    return std::cos(x);
  }
  int quo;
  double x90 = std::remquo(std::fabs(x), 90.0, &quo);
  double xr = d2r(x90);
  switch (quo % 4) {
    case 0:
      return std::cos(xr);
    case 1:
      // Use + 0.0 to avoid -0.0
      return std::sin(-xr + 0.0);
    case 2:
      return -std::cos(xr);
    case 3:
      return std::sin(xr + 0.0);
  }
  return 0.0;
}

void openAuxDevice(int index = 0)
{
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (0, &devs); //get the list of devices
	if (cnt < 0)
	{
		ROS_ERROR("No device on USB");
		return;
	}
	
	int nr_mot(0);
	for (int i = 0; i < cnt; ++i)
	{
		struct libusb_device_descriptor desc;
		const int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;

		// Search for the aux
		if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
		{
			// If the index given by the user matches our camera index
			if (nr_mot == index)
			{
				if ((libusb_open (devs[i], &dev) != 0) || (dev == 0))
				{
					ROS_ERROR_STREAM("Cannot open aux " << index);
					return;
				}
				// Claim the aux
				libusb_claim_interface (dev, 0);
				break;
			}
			else
				nr_mot++;
		}
	}

	libusb_free_device_list (devs, 1);  // free the list, unref the devices in it
}

void setTiltAngle(const std_msgs::Float64 angleMsg)
{
	uint8_t empty[0x1];
	double angle(angleMsg.data);

	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
	angle = angle * 2;
	if (!use_imu_for_angle) {
		old_tilt_angle = angle;
	}
	const int ret = libusb_control_transfer(dev, 0x40, 0x31, (int16_t)angle, 0x0, empty, 0x0, 0);
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting tilt angle, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
}

void setLedOption(const std_msgs::UInt16 optionMsg)
{
	uint8_t empty[0x1];
	const uint16_t option(optionMsg.data);
	
	const int ret = libusb_control_transfer(dev, 0x40, 0x06, (uint16_t)option, 0x0, empty, 0x0, 0);
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting LED options, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
}

void publishState(void)
{
	uint8_t buf[10];
	const int ret = libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0);
	if (ret != 10)
	{
		ROS_ERROR_STREAM("Error in accelerometer reading, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}

	if (!has_homed) {
		std_msgs::Float64 home_tilt_msg;
		home_tilt_msg.data = init_tilt_angle;
		setTiltAngle(home_tilt_msg);
		has_homed = true;
	}
	
	const uint16_t ux = ((uint16_t)buf[2] << 8) | buf[3];
	const uint16_t uy = ((uint16_t)buf[4] << 8) | buf[5];
	const uint16_t uz = ((uint16_t)buf[6] << 8) | buf[7];
	
	const int16_t accelerometer_x = (int16_t)ux;
	const int16_t accelerometer_y = (int16_t)uy;
	const int16_t accelerometer_z = (int16_t)uz;
	const int8_t tilt_angle = (int8_t)buf[8];
	const uint8_t tilt_status = buf[9];
	
	// publish IMU
	sensor_msgs::Imu imu_msg;
	if (pub_imu.getNumSubscribers() > 0)
	{
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.linear_acceleration.x = (double(accelerometer_x)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration.y = (double(accelerometer_y)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration.z = (double(accelerometer_z)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4]
			= imu_msg.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
		imu_msg.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
		imu_msg.orientation_covariance[0] = -1; // indicates orientation not provided
		pub_imu.publish(imu_msg);
	}
	
	// publish tilt angle and status
	if (pub_tilt_angle.getNumSubscribers() > 0)
	{
		std_msgs::Float64 tilt_angle_msg;
		tilt_angle_msg.data = double(tilt_angle) / 2.;
		pub_tilt_angle.publish(tilt_angle_msg);
	}
	if (pub_tilt_status.getNumSubscribers() > 0)
	{
		std_msgs::UInt8 tilt_status_msg;
		tilt_status_msg.data = tilt_status;
		pub_tilt_status.publish(tilt_status_msg);
	}
    
	if (pub_tf || pub_joint_state) 
	{
		double current_tilt;
		if (use_imu_for_angle)
		{
			// tilt_status of 4 seems to mean that it is moving
			current_tilt = tilt_angle;
			if (tilt_status == 4) {
				current_tilt = old_tilt_angle;
			} else {
				old_tilt_angle = tilt_angle;
			}
		} else {
			current_tilt = old_tilt_angle;
		}
	    //Polar coordinates for kinect_base to camera_link is 0.01818<68.37deg
	    
	    double cam_angle = double(current_tilt) / 2.;
		if (cam_angle <= -44) {
			cam_angle = -44;
		}

        if (pub_tf) {
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			double cam_x = cosd(cam_angle + 68.37) * 0.01818;
			double cam_y = std::sin(d2r(cam_angle + 68.37)) * 0.01818;
			transform.setOrigin( tf::Vector3(cam_x, 0.0117, cam_y) );
			tf::Quaternion q;
			double angleRad = d2r(cam_angle * -1); // * 0.01745329;
		
			q.setRPY(0, angleRad, 0);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), kinect_pivot_tf_name, kinect_camera_tf_name));
		} 
		
		if (pub_joint_state && joint_pub.getNumSubscribers() > 0) {
			sensor_msgs::JointState joint_state;
			joint_state.header.stamp = ros::Time::now();
			joint_state.name.resize(1);
            joint_state.position.resize(1);
			joint_state.name[0] =kinect_pivot_joint_name;
			double angleRad = d2r(cam_angle * -1); // * 0.01745329;
			joint_state.position[0] = angleRad;
			joint_pub.publish(joint_state);
		}
    }

	
}





int main(int argc, char* argv[])
{
	int ret = libusb_init(0);
	if (ret)
	{
		ROS_ERROR_STREAM("Cannot initialize libusb, error: " << ret);
		return 1;
	}
	
	ros::init(argc, argv, "kinect_aux");
	ros::NodeHandle n, pnh("~");
	
	int deviceIndex;

	

	n.param<int>("device_index", deviceIndex, 0);
    pnh.param<int>("max_tilt_angle", MAX_TILT_ANGLE, 31);
	pnh.param<int>("min_tilt_angle", MIN_TILT_ANGLE, (-31));
	pnh.param<bool>("pub_tf", pub_tf, false);
	pnh.param<bool>("pub_joint_state", pub_joint_state, false);
	pnh.param<std::string>("pivot_tf_frame", kinect_pivot_tf_name, "kinect_pivot");
	pnh.param<std::string>("camera_tf_frame", kinect_camera_tf_name, "camera_link");
	pnh.param<std::string>("pivot_joint_name", kinect_pivot_joint_name, "kinect_pivot_to_camera_link");
	pnh.param<bool>("use_imu_for_angle", use_imu_for_angle, false);
	pnh.param<int>("init_tilt_angle", init_tilt_angle, 0);
	

	openAuxDevice(deviceIndex);
	if (!dev)
	{
		ROS_ERROR_STREAM("No valid aux device found");
		libusb_exit(0);
		return 2;
	}

	if (pub_joint_state){
		joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	}
	
	pub_imu = n.advertise<sensor_msgs::Imu>("imu", 15);
	pub_tilt_angle = n.advertise<std_msgs::Float64>("cur_tilt_angle", 15);
	pub_tilt_status = n.advertise<std_msgs::UInt8>("cur_tilt_status", 15);
	
	sub_tilt_angle = n.subscribe("tilt_angle", 1, setTiltAngle);
	sub_led_option = n.subscribe("led_option", 1, setLedOption);
	 
	while (ros::ok())
	{
		ros::spinOnce();
		publishState();
	}
	
	libusb_exit(0);
	return 0;
}