// Returns 1 if `pid` identifies K4W audio, 0 otherwise
int fnusb_is_pid_k4w_audio(int pid)
{
	return (pid == PID_K4W_AUDIO || pid == PID_K4W_AUDIO_ALT_1 || pid == PID_K4W_AUDIO_ALT_2);
}

int fnusb_num_devices(fnusb_ctx *ctx)
{
	libusb_device **devs;
	//pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (ctx->ctx, &devs);
	//get the list of devices
	if (cnt < 0)
		return (-1);
	int nr = 0, i = 0;
	struct libusb_device_descriptor desc;
	for (i = 0; i < cnt; ++i)
	{
		int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;
		if (desc.idVendor == VID_MICROSOFT && (desc.idProduct == PID_NUI_CAMERA || desc.idProduct == PID_K4W_CAMERA))
			nr++;
	}
	libusb_free_device_list (devs, 1);
	// free the list, unref the devices in it
	return nr;
}
