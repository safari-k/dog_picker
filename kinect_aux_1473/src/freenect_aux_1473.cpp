#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <libusb.h>
#include "freenect_aux_1473.h"
//#include <libfreenect/libfreenect.h>
//#include "freenectaudio.hpp"
//#include "/usr/local/include/libfreenect/libfreenect.h"
//#include "/usr/local/include/libfreenect/libfreenect.hpp"

// Returns 1 if `pid` identifies K4W audio, 0 otherwise
int fnusb_is_pid_k4w_audio(int pid)
{
	return (pid == PID_K4W_AUDIO || pid == PID_K4W_AUDIO_ALT_1 || pid == PID_K4W_AUDIO_ALT_2);
}

int fnusb_open_audiodevice(freenect_device *dev, int index)
{
	freenect_context *ctx = dev->parent;

	dev->device_does_motor_control_with_audio = 1; //1473 always
	dev->motor_control_with_audio_enabled = 0;

	dev->usb_motor.parent = dev;
	dev->usb_motor.dev = NULL;
	dev->usb_audio.parent = dev;
	dev->usb_audio.dev = NULL;

	libusb_device **devs; // pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (dev->parent->usb.ctx, &devs); //get the list of devices
	if (cnt < 0)
		return -1;

	int i = 0, nr_cam = 0, nr_mot = 0;
	int nr_audio = 0;
	int res;
	struct libusb_device_descriptor desc;

	// Search for the motor
	for (i = 0; i < cnt; i++)
	{
		int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;

		if (desc.idVendor != VID_MICROSOFT)
			continue;
		if ((ctx->enabled_subdevices & FREENECT_DEVICE_MOTOR) && !dev->usb_motor.dev && desc.idProduct == PID_NUI_MOTOR)
		{
			// If the index given by the user matches our camera index
			if (nr_mot == index)
			{
				dev->usb_motor.VID = desc.idVendor;
				dev->usb_motor.PID = desc.idProduct;

				res = libusb_open (devs[i], &dev->usb_motor.dev);
				if (res < 0 || !dev->usb_motor.dev)
				{
					FN_ERROR("Could not open motor: %d\n", res);
					dev->usb_motor.dev = NULL;
					break;
				}
				res = libusb_claim_interface (dev->usb_motor.dev, 0);
				if (res < 0)
				{
					FN_ERROR("Could not claim interface on motor: %d\n", res);
					libusb_close(dev->usb_motor.dev);
					dev->usb_motor.dev = NULL;
					break;
				}
			}
			else
			{
				nr_mot++;
			}
		}

		// Search for the audio
		if ((ctx->enabled_subdevices & FREENECT_DEVICE_AUDIO) && !dev->usb_audio.dev && (desc.idProduct == PID_NUI_AUDIO || fnusb_is_pid_k4w_audio(desc.idProduct)))
		{
			// If the index given by the user matches our audio index
			if (nr_audio == index)
			{
				dev->usb_audio.VID = desc.idVendor;
				dev->usb_audio.PID = desc.idProduct;

				res = libusb_open (devs[i], &dev->usb_audio.dev);
				if (res < 0 || !dev->usb_audio.dev)
				{
					FN_ERROR("Could not open audio: %d\n", res);
					dev->usb_audio.dev = NULL;
					break;
				}

				res = libusb_claim_interface (dev->usb_audio.dev, 0);
				if (res < 0)
				{
					FN_ERROR("Could not claim interface on audio: %d\n", res);
					libusb_close(dev->usb_audio.dev);
					dev->usb_audio.dev = NULL;
					break;
				}

				// Using the device handle that we've claimed, see if this
				// device has already uploaded firmware (has 2 interfaces).
				// If not, save the serial number (by reading the appropriate
				// descriptor), upload the firmware, and then enter a loop
				// waiting for a device with the same serial number to
				// reappear.
				int num_interfaces = fnusb_num_interfaces(&dev->usb_audio);

				if (num_interfaces >= 2)
				{
				    FN_SPEW("2 interfaces\n");
					if (dev->device_does_motor_control_with_audio)
					{
						dev->motor_control_with_audio_enabled = 1;
						FN_SPEW("device_does_motor_control_with_audio\n");
					}
				}
				else
				{
					// Read the serial number from the string descriptor and save it.
					unsigned char string_desc[256]; // String descriptors are at most 256 bytes
					res = libusb_get_string_descriptor_ascii(dev->usb_audio.dev, desc.iSerialNumber, string_desc, 256);
					if (res < 0)
					{
						FN_ERROR("Failed to retrieve serial number for audio device in bootloader state\n");
						break;
					}
					char* audio_serial = strdup((char*)string_desc);

					FN_SPEW("Uploading firmware to audio device in bootloader state.\n");

					// Check if we can load from memory - otherwise load from disk
					if (desc.idProduct == PID_NUI_AUDIO && ctx->fn_fw_nui_ptr && ctx->fn_fw_nui_size > 0)
					{
						FN_SPEW("loading firmware from memory\n");
						res = upload_firmware_from_memory(&dev->usb_audio, ctx->fn_fw_nui_ptr, ctx->fn_fw_nui_size);
					}
					else if (desc.idProduct == PID_K4W_AUDIO && ctx->fn_fw_k4w_ptr && ctx->fn_fw_k4w_size > 0)
					{
						FN_SPEW("loading firmware from memory\n");
						res = upload_firmware_from_memory(&dev->usb_audio, ctx->fn_fw_k4w_ptr, ctx->fn_fw_k4w_size);
					}
					else
					{
						res = upload_firmware(&dev->usb_audio, "audios.bin");
					}

					if (res < 0)
					{
						FN_ERROR("upload_firmware failed: %d\n", res);
						break;
					}
					libusb_close(dev->usb_audio.dev);
					dev->usb_audio.dev = NULL;
					// Wait for the device to reappear.
					int loops = 0;
					for (loops = 0; loops < 10; loops++)
					{
						FN_SPEW("Try %d: Looking for new audio device matching serial %s\n", loops, audio_serial);
						// Scan devices.
						libusb_device **new_dev_list;
						int dev_index;
						ssize_t num_new_devs = libusb_get_device_list(ctx->usb.ctx, &new_dev_list);
						for (dev_index = 0; dev_index < num_new_devs; ++dev_index)
						{
							struct libusb_device_descriptor new_dev_desc;
							int r;
							r = libusb_get_device_descriptor (new_dev_list[dev_index], &new_dev_desc);
							if (r < 0)
								continue;
							// If this dev is a Kinect audio device, open device, read serial, and compare.
							if (new_dev_desc.idVendor == VID_MICROSOFT && (new_dev_desc.idProduct == PID_NUI_AUDIO || fnusb_is_pid_k4w_audio(desc.idProduct)))
							{
								FN_SPEW("Matched VID/PID!\n");
								libusb_device_handle* new_dev_handle;
								// Open device
								r = libusb_open(new_dev_list[dev_index], &new_dev_handle);
								if (r < 0)
									continue;
								// Read serial
								r = libusb_get_string_descriptor_ascii(new_dev_handle, new_dev_desc.iSerialNumber, string_desc, 256);
								if (r < 0)
								{
									FN_SPEW("Lost new audio device while fetching serial number.\n");
									libusb_close(new_dev_handle);
									continue;
								}
								// Compare to expected serial
								if (r == strlen(audio_serial) && strcmp((char*)string_desc, audio_serial) == 0)
								{
									// We found it!
									r = libusb_claim_interface(new_dev_handle, 0);
									if (r != 0)
									{
										// Ouch, found the device but couldn't claim the interface.
										FN_SPEW("Device with serial %s reappeared but couldn't claim interface 0\n", audio_serial);
										libusb_close(new_dev_handle);
										continue;
									}
									// Save the device handle.
									dev->usb_audio.dev = new_dev_handle;

									// Verify that we've actually found a device running the right firmware.
									num_interfaces = fnusb_num_interfaces(&dev->usb_audio);

									if (num_interfaces >= 2)
									{
										if (dev->device_does_motor_control_with_audio)
										{
											dev->motor_control_with_audio_enabled = 1;
										}
									}
									else
									{
										FN_SPEW("Opened audio with matching serial but too few interfaces.\n");
										dev->usb_audio.dev = NULL;
										libusb_close(new_dev_handle);
										continue;
									}

									break;
								}
								else
								{
									FN_SPEW("Got serial %s, expected serial %s\n", (char*)string_desc, audio_serial);
								}
							}
						}

						libusb_free_device_list(new_dev_list, 1);
						// If we found the right device, break out of this loop.
						if (dev->usb_audio.dev)
							break;
						// Sleep for a second to give the device more time to reenumerate.
						sleep(1);
					}
					free(audio_serial);
				}
			}
			else
			{
				nr_audio++;
			}
		}
	}

	libusb_free_device_list (devs, 1);  // free the list, unref the devices in it

	if  ((dev->usb_motor.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_MOTOR))
		&& (dev->usb_audio.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_AUDIO)))
	{
		// Each requested subdevice is open.
		// Except audio, which may fail if firmware is missing (or because it hates us).
		return 0;
	}
/*
	if (dev->usb_motor.dev != NULL)
	{
		libusb_release_interface(dev->usb_motor.dev, 0);
		libusb_close(dev->usb_motor.dev);
	}
	else
	{
		FN_ERROR("Failed to open motor subddevice or it is not disabled.");
	}
*/
	if (dev->usb_audio.dev != NULL)
	{
		libusb_release_interface(dev->usb_audio.dev, 0);
		libusb_close(dev->usb_audio.dev);
		FN_SPEW("Found and released audio\n");
	}
	else
	{
		FN_ERROR("Failed to open audio subdevice or it is not disabled.");
	}

	return -1;
}

int freenect_open_audiodevice(freenect_context *ctx, freenect_device **dev, int index)
{
	int res;
	freenect_device *pdev = (freenect_device*)malloc(sizeof(freenect_device));
	if (!pdev)
		return -1;

	memset(pdev, 0, sizeof(*pdev));

	pdev->parent = ctx;

	res = fnusb_open_audiodevice(pdev, index);
	if (res < 0) {
		free(pdev);
		return res;
	}

	if (!ctx->first) {
		ctx->first = pdev;
	} else {
		freenect_device *prev = ctx->first;
		while (prev->next)
			prev = prev->next;
		prev->next = pdev;
	}

	*dev = pdev;

	/* Do device-specific initialization
	if (pdev->usb_cam.dev) {
		if (freenect_camera_init(pdev) < 0) {
			return -1;
		}
	}
    */
	return 0;
}
void fn_log(freenect_context *ctx, freenect_loglevel level, const char *fmt, ...)
{
	va_list ap;

	if (level > ctx->log_level)
		return;

	if (ctx->log_cb) {
		char msgbuf[1024];

		va_start(ap, fmt);
		vsnprintf(msgbuf, 1024, fmt, ap);
		msgbuf[1023] = 0;
		va_end(ap);

		ctx->log_cb(ctx, level, msgbuf);
	} else {
		va_start(ap, fmt);
		vfprintf(stderr, fmt, ap);
		va_end(ap);
	}
}


// in kinect 1473 the motor/accelerometer is connected to the audio
class RosFreenectAudioNode {
  public:
    RosFreenectAudioNode(ros::NodeHandle* n) {
        _n = n;
        timeout = { 1, 0 };

        int ret = freenect_init(&ctx, 0);
        if(ret < 0) {
            ROS_ERROR_STREAM("No valid Audio/Motor device found\n");
            libusb_exit(0);
        }
        freenect_set_log_level(ctx, LL_SPEW );
        ROS_DEBUG("Got context\n");
        freenect_select_subdevices(ctx, static_cast<freenect_device_flags> (FREENECT_DEVICE_AUDIO));
        ret = freenect_open_audiodevice(ctx, &dev, 0);
        if(ret < 0) {
            ROS_ERROR_STREAM("Unable to open Audio/Motor device\n");
            libusb_exit(0);
        }
        ROS_INFO("Got device\n");

        pub_imu = n->advertise<sensor_msgs::Imu>("imu", 15);
        pub_tilt_angle = _n->advertise<std_msgs::Float64>("cur_tilt_angle", 15);
        pub_tilt_status = _n->advertise<std_msgs::UInt8>("cur_tilt_status", 15);
        // Name the topic, message queue, callback function with class name, and object containing callback function.
        sub_tilt_angle = _n->subscribe("tilt_angle", 1, &RosFreenectAudioNode::setTiltAngle, this);
        sub_led_option = _n->subscribe("led_option", 1, &RosFreenectAudioNode::setLedOption, this);

    }
    ~RosFreenectAudioNode(){
        libusb_exit(0);
        ros::shutdown();
    }

    void publishState()
    {
        freenect_raw_tilt_state *state = 0;
        double dx, dy, dz;
        //static timeval timeout = { 1, 0 };

        freenect_update_tilt_state(dev);
        state = freenect_get_tilt_state(dev);
        freenect_process_events_timeout(ctx,&timeout);
        freenect_get_mks_accel(state, &dx, &dy, &dz);

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
        int ret = freenect_set_tilt_degs(dev, (double) angle);
        if (ret != 0)
        {
            ROS_ERROR_STREAM("Error in setting tilt angle, freenect_sync_set_tilt_degs returned " << ret);
            ros::shutdown();
        }
    }

    void setLedOption(const std_msgs::UInt16 optionMsg)
    {
        const uint16_t option(optionMsg.data);
        freenect_led_options led = (freenect_led_options) ((uint16_t)option % 6); // explicit cast
        int ret = freenect_set_led(dev, led);
        // Set the LEDs to one of the possible states
        if (ret!= 0)
        {
            ROS_ERROR_STREAM("Error in setting LED options, freenect_sync_set_led returned " << ret);
            ros::shutdown();
        }
    }

  protected:
  private:
    freenect_context *ctx;
    freenect_device* dev;
    freenect_tilt_status_code m_code;
    freenect_raw_tilt_state *state;
    timeval timeout;

    ros::Publisher pub_imu;
    ros::Publisher pub_tilt_angle;
    ros::Publisher pub_tilt_status;

    ros::Subscriber sub_tilt_angle;
    ros::Subscriber sub_led_option;
    ros::NodeHandle* _n;
};

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "~");
	ros::NodeHandle n;
    RosFreenectAudioNode auxnode(&n);

	while (ros::ok())
	{
		ros::spinOnce();
		auxnode.publishState();
	}

	return 0;
}