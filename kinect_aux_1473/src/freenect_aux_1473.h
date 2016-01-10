
#include "/usr/local/include/libfreenect/libfreenect.h"
#include "/usr/local/include/libfreenect/libfreenect_audio.h"
#include "/usr/local/include/libfreenect/libfreenect_registration.h"
#include "freenect_internal.h"
#include "loader.h"
#include "usb_libusb10.h"

// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 31.
#define MIN_TILT_ANGLE (-31.)
