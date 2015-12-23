#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <libfreenect/libfreenect.h>

class FreenectTiltState {
  public:
    FreenectTiltState(freenect_raw_tilt_state *_state):
        m_code(_state->tilt_status), m_state(_state)
    {}

    void getAccelerometers(double* x, double* y, double* z) {
        freenect_get_mks_accel(m_state, x, y, z);
    }
    double getTiltDegs() {
        return freenect_get_tilt_degs(m_state);
    }
  public:
    freenect_tilt_status_code m_code;
  public:
    freenect_raw_tilt_state *m_state;
};

int main(int argc, char* argv[])
{

	//ros::init(argc, argv, "kinect_aux_1473");
	//ros::NodeHandle n;

    freenect_context *ctx;
    int ret = freenect_init(&ctx, 0);
    if(ret < 0)
        return 1;
    printf("got context\n");
    //freenect_select_subdevices(ctx, static_cast<freenect_device_flags> (FREENECT_DEVICE_AUDIO));
    freenect_device* dev;
    //dev.motor_control_with_audio_enabled = 1;
    ret = freenect_open_device(ctx, &dev, 0);
    if(ret < 0)
        return 2;
    printf("got device\n");
    static timeval timeout = { 1, 0 };

    freenect_update_tilt_state(dev);
    freenect_process_events_timeout(ctx,&timeout);
    FreenectTiltState t_state = FreenectTiltState(freenect_get_tilt_state(dev));
    freenect_process_events_timeout(ctx,&timeout);
    printf("tilt %f \n",t_state.getTiltDegs());
    double dx, dy, dz;
    t_state.getAccelerometers(&dx,&dy,&dz);
    printf("accel[%lf,%lf,%lf]\n", dx,dy,dz);

    freenect_set_led(dev, LED_RED);

	freenect_raw_tilt_state *state = 0;
	freenect_update_tilt_state(dev);
	state = freenect_get_tilt_state(dev);
	freenect_process_events_timeout(ctx,&timeout);
	freenect_get_mks_accel(state, &dx, &dy, &dz);
	//usleep(10000000);
	printf("tilt[%d] accel[%lf,%lf,%lf]\n",state->tilt_angle, dx,dy,dz);
	printf("tilt[%d] accel[%d,%d,%d]\n",state->tilt_angle, state->accelerometer_x,state->accelerometer_y,state->accelerometer_z);

/*	while( !n.hasParam("initialized") ){
        ros::spinOnce();
    }
    bool b = false;
	while( !b ){
	    n.param("initialized",b, false);
        ros::spinOnce();
    }
    ros::Time time = ros::Time::now();
    //Wait a duration of 5 second.
    ros::Duration d = ros::Duration(5, 0);
    d.sleep();

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
*/
	return 0;
}