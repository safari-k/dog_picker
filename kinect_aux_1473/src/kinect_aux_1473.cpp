#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>

// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 31.
#define MIN_TILT_ANGLE (-31.)

ros::Publisher pub_imu;
ros::Publisher pub_tilt_angle;
ros::Publisher pub_tilt_status;

ros::Subscriber sub_tilt_angle;
ros::Subscriber sub_led_option;

/*
extern int freenect_sync_get_tilt_state(freenect_raw_tilt_state **state, int index);
extern int freenect_sync_set_tilt_degs(int angle, int index);
extern int freenect_sync_set_led(freenect_led_options led, int index);
extern void freenect_sync_stop(void);
*/

void publishState(void)
{
	freenect_raw_tilt_state *state = 0;
	double dx, dy, dz;
	int ret = freenect_sync_get_tilt_state(&state, 0);
	if (ret ==0)
	{
		freenect_get_mks_accel(state, &dx, &dy, &dz);
		//printf("tilt[%d] accel[%lf,%lf,%lf]\n",state->tilt_angle, dx,dy,dz);
	}

	// publish IMU
	sensor_msgs::Imu imu_msg;
	if (pub_imu.getNumSubscribers() > 0)
	{
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.linear_acceleration.x = dx;
		imu_msg.linear_acceleration.y = dy;
		imu_msg.linear_acceleration.z = dz;
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
		tilt_angle_msg.data = state->tilt_angle;
		pub_tilt_angle.publish(tilt_angle_msg);
	}
	if (pub_tilt_status.getNumSubscribers() > 0)
	{
		std_msgs::UInt8 tilt_status_msg;
		tilt_status_msg.data = state->tilt_status;
		pub_tilt_status.publish(tilt_status_msg);
	}
}


void setTiltAngle(const std_msgs::Float64 angleMsg)
{
	int angle(angleMsg.data);
	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
	int ret = freenect_sync_set_tilt_degs(angle, 0);
	if (ret != 0)
	{
		freenect_sync_stop();
		ROS_ERROR_STREAM("Error in setting tilt angle, freenect_sync_set_tilt_degs returned " << ret);
		ros::shutdown();
	}
}

void setLedOption(const std_msgs::UInt16 optionMsg)
{
	const uint16_t option(optionMsg.data);
	freenect_led_options led = (freenect_led_options) ((uint16_t)option % 6); // explicit cast
	int ret = freenect_sync_set_led(led, 0);
	// Set the LEDs to one of the possible states
	if (ret!= 0)
	{
		freenect_sync_stop();
		ROS_ERROR_STREAM("Error in setting LED options, freenect_sync_set_led returned " << ret);
		ros::shutdown();
	}
}


int main(int argc, char* argv[])
{
	/*
	int ret = freenect_num_devices(freenect_context *ctx);

	int ret = libusb_init(0);
	if (ret)
	{
		ROS_ERROR_STREAM("Cannot initialize libusb, error: " << ret);
		return 1;
	}
	
	ros::init(argc, argv, "kinect_aux_1473");
	ros::NodeHandle n;
	
	int deviceIndex;
	n.param<int>("device_index", deviceIndex, 0);
	openAuxDevice(deviceIndex);
	if (!dev)
	{
		ROS_ERROR_STREAM("No valid aux device found");
		libusb_exit(0);
		return 2;
	}
	*/
	ros::init(argc, argv, "kinect_aux_1473");
	ros::NodeHandle n;	
	
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
	
	return 0;
}
