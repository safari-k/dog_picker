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


int main(int argc, char* argv[])
{

	//ros::init(argc, argv, "kinect_aux_1473");
	//ros::NodeHandle n;

    freenect_context *ctx;
    int ret = freenect_init(&ctx, 0);
    if(ret < 0)
        return 1;
    //ctx->log_level = LL_SPEW;
    freenect_set_log_level(ctx, LL_SPEW );
    printf("got context\n");
    freenect_select_subdevices(ctx, static_cast<freenect_device_flags> (FREENECT_DEVICE_AUDIO));
    freenect_device* dev;
    //dev->motor_control_with_audio_enabled = 1;
    //dev->device_does_motor_control_with_audio = 1;
    //ret = freenect_open_device(ctx, &dev, 0);
    ret = freenect_open_audiodevice(ctx, &dev, 0);
    if(ret < 0) {
        FN_SPEW("freenect_open_audiodevice ret =%d\n", ret);
        return 2;
    }
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

    freenect_set_led(dev, LED_GREEN);

	freenect_raw_tilt_state *state = 0;
	freenect_update_tilt_state(dev);
	state = freenect_get_tilt_state(dev);
	freenect_process_events_timeout(ctx,&timeout);
	freenect_get_mks_accel(state, &dx, &dy, &dz);
	//usleep(10000000);
	printf("tilt[%d] accel[%lf,%lf,%lf]\n",state->tilt_angle, dx,dy,dz);
	printf("tilt[%d] accel[%d,%d,%d]\n",state->tilt_angle, state->accelerometer_x,state->accelerometer_y,state->accelerometer_z);

    //Freenect::FreenectAudio freenect;
    //Freenect::FreenectDevice* device;
	//device = &freenect.createDevice<Freenect::FreenectDevice>(0);

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