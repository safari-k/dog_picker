#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb.h>

#include <libfreenect/libfreenect.h>

int freenect_open_device(freenect_context *ctx, freenect_device **dev, int index)
{
	int res;
	freenect_device *pdev = (freenect_device*)malloc(sizeof(freenect_device));
	if (!pdev)
		return -1;

	memset(pdev, 0, sizeof(*pdev));

	pdev->parent = ctx;

	res = fnusb_open_subdevices(pdev, index);
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
int fnusb_open_subdevices(freenect_device *dev, int index)
{
	freenect_context *ctx = dev->parent;

	dev->device_does_motor_control_with_audio = 0;
	dev->motor_control_with_audio_enabled = 0;

	dev->usb_cam.parent = dev;
	dev->usb_cam.dev = NULL;
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

	for (i = 0; i < cnt; i++)
	{
		int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;

		if (desc.idVendor != VID_MICROSOFT)
			continue;
		res = 0;
		// Search for the camera
		if ((ctx->enabled_subdevices & FREENECT_DEVICE_CAMERA) && !dev->usb_cam.dev && (desc.idProduct == PID_NUI_CAMERA || desc.idProduct == PID_K4W_CAMERA))
		{
			// If the index given by the user matches our camera index
			if (nr_cam == index)
			{
				dev->usb_cam.VID = desc.idVendor;
				dev->usb_cam.PID = desc.idProduct;

				res = libusb_open (devs[i], &dev->usb_cam.dev);
				if (res < 0 || !dev->usb_cam.dev)
				{
					FN_ERROR("Could not open camera: %d\n", res);
					dev->usb_cam.dev = NULL;
					break;
				}

				if (desc.idProduct == PID_K4W_CAMERA || desc.bcdDevice != fn_le32(267))
				{
					freenect_device_flags requested_devices = ctx->enabled_subdevices;

					// Not the 1414 kinect so remove the motor flag, this should preserve the audio flag if set
					ctx->enabled_subdevices = (freenect_device_flags)(ctx->enabled_subdevices & ~FREENECT_DEVICE_MOTOR);

					ctx->zero_plane_res = 334;
					dev->device_does_motor_control_with_audio = 1;

					// set the LED for non 1414 devices to keep the camera alive for some systems which get freezes

					libusb_device * audioDevice = fnusb_find_connected_audio_device(devs[i], devs, cnt);
					if (audioDevice != NULL)
					{
						libusb_device_handle * audioHandle = NULL;
						res = libusb_open(audioDevice, &audioHandle);

						if (res != 0)
						{
							FN_ERROR("Failed to set the LED of K4W or 1473 device: %d\n", res);
						}
						else
						{
							// we need to do this as it is possible that the device was not closed properly in a previous session
							// if we don't do this and the device wasn't closed properly - it can cause infinite hangs on LED and TILT functions
							libusb_reset_device(audioHandle);
							libusb_close(audioHandle);

							res = libusb_open(audioDevice, &audioHandle);
							if (res == 0)
							{
								res = libusb_claim_interface(audioHandle, 0);
								if (res != 0)
								{
									FN_ERROR("Unable to claim interface %d\n", res);
								}
								else
								{
									fnusb_set_led_alt(audioHandle, ctx, LED_GREEN);
									libusb_release_interface(audioHandle, 0);
								}
								libusb_close(audioHandle);
							}
						}
					}
					// for newer devices we need to enable the audio device for motor control
					// we only do this though if motor has been requested.
					if ((requested_devices & FREENECT_DEVICE_MOTOR) && (requested_devices & FREENECT_DEVICE_AUDIO) == 0)
					{
						ctx->enabled_subdevices = (freenect_device_flags)(ctx->enabled_subdevices | FREENECT_DEVICE_AUDIO);
					}
				}
				else
				{
					// The good old kinect that tilts and tweets
					ctx->zero_plane_res = 322;
				}

				res = fnusb_claim_camera(dev);
				if (res < 0)
				{
					break;
				}
			}
			else
			{
				nr_cam++;
			}
		}
	}

	if (ctx->enabled_subdevices == FREENECT_DEVICE_CAMERA || res < 0)
		cnt = 0;

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
					if (dev->device_does_motor_control_with_audio)
					{
						dev->motor_control_with_audio_enabled = 1;
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

	if ((dev->usb_cam.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_CAMERA))
   && (dev->usb_motor.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_MOTOR)))
		//&& (dev->usb_audio.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_AUDIO)))
	{
		// Each requested subdevice is open.
		// Except audio, which may fail if firmware is missing (or because it hates us).
		return 0;
	}

	if (dev->usb_cam.dev != NULL)
	{
		libusb_release_interface(dev->usb_cam.dev, 0);
		libusb_close(dev->usb_cam.dev);
	}
	else
	{
		FN_ERROR("Failed to open camera subdevice or it is not disabled.");
	}

	if (dev->usb_motor.dev != NULL)
	{
		libusb_release_interface(dev->usb_motor.dev, 0);
		libusb_close(dev->usb_motor.dev);
	}
	else
	{
		FN_ERROR("Failed to open motor subddevice or it is not disabled.");
	}

	if (dev->usb_audio.dev != NULL)
	{
		libusb_release_interface(dev->usb_audio.dev, 0);
		libusb_close(dev->usb_audio.dev);
	}
	else
	{
		FN_ERROR("Failed to open audio subdevice or it is not disabled.");
	}

	return -1;
}
