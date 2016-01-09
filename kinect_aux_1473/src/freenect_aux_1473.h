
#include "/usr/local/include/libfreenect/libfreenect.h"
#include "/usr/local/include/libfreenect/libfreenect_audio.h"
#include "/usr/local/include/libfreenect/libfreenect_registration.h"
#include "freenect_internal.h"
#include "loader.h"
#include "usb_libusb10.h"
/*
typedef void (*fnusb_iso_cb)(freenect_device *dev, uint8_t *buf, int len);

//#include "usb_libusb10.h"

typedef struct {
	libusb_context *ctx;
	int should_free_ctx;
} fnusb_ctx;

typedef struct {
	freenect_device *parent; //so we can go up from the libusb userdata
	libusb_device_handle *dev;
	int device_dead; // set to 1 when the underlying libusb_device_handle vanishes (ie, Kinect was unplugged)
	int VID;
	int PID;
} fnusb_dev;

typedef struct {
	fnusb_dev *parent; //so we can go up from the libusb userdata
	struct libusb_transfer **xfers;
	uint8_t *buffer;
	fnusb_iso_cb cb;
	int num_xfers;
	int pkts;
	int len;
	int dead;
	int dead_xfers;
} fnusb_isoc_stream;

struct _freenect_context {
	freenect_loglevel log_level;
	freenect_log_cb log_cb;
	fnusb_ctx usb;
	freenect_device_flags enabled_subdevices;
	freenect_device *first;
	int zero_plane_res;

    // if you want to load firmware from memory rather than disk
    unsigned char *     fn_fw_nui_ptr;
    unsigned int        fn_fw_nui_size;

    unsigned char *     fn_fw_k4w_ptr;
    unsigned int        fn_fw_k4w_size;
};

#define LL_FATAL FREENECT_LOG_FATAL
#define LL_ERROR FREENECT_LOG_ERROR
#define LL_WARNING FREENECT_LOG_WARNING
#define LL_NOTICE FREENECT_LOG_NOTICE
#define LL_INFO FREENECT_LOG_INFO
#define LL_DEBUG FREENECT_LOG_DEBUG
#define LL_SPEW FREENECT_LOG_SPEW
#define LL_FLOOD FREENECT_LOG_FLOOD


#ifdef _WIN32
#include <stdarg.h>
#include <stdio.h>
void fn_log(freenect_context *ctx, freenect_loglevel level, const char *fmt, ...);
#else
void fn_log(freenect_context *ctx, freenect_loglevel level, const char *fmt, ...) __attribute__ ((format (printf, 3, 4)));
#endif

#define FN_LOG(level, ...) fn_log(ctx, level, __VA_ARGS__)

#define FN_FATAL(...) FN_LOG(LL_FATAL, __VA_ARGS__)
#define FN_ERROR(...) FN_LOG(LL_ERROR, __VA_ARGS__)
#define FN_WARNING(...) FN_LOG(LL_WARNING, __VA_ARGS__)
#define FN_NOTICE(...) FN_LOG(LL_NOTICE, __VA_ARGS__)
#define FN_INFO(...) FN_LOG(LL_INFO, __VA_ARGS__)
#define FN_DEBUG(...) FN_LOG(LL_DEBUG, __VA_ARGS__)
#define FN_SPEW(...) FN_LOG(LL_SPEW, __VA_ARGS__)
#define FN_FLOOD(...) FN_LOG(LL_FLOOD, __VA_ARGS__)

#define VID_MICROSOFT 0x45e
#define PID_NUI_AUDIO 0x02ad
#define PID_NUI_CAMERA 0x02ae
#define PID_NUI_MOTOR 0x02b0
#define PID_K4W_CAMERA 0x02bf

// For K4W: first pid is what it starts out as,
// second is how it appears with lastest firmware from SDK,
// third is from beta SDK firmware ( which is what is unpacked by the fw script and doesn't support motor control )
#define PID_K4W_AUDIO 0x02be
#define PID_K4W_AUDIO_ALT_1 0x02c3
#define PID_K4W_AUDIO_ALT_2 0x02bb

typedef struct {
	int running;
	uint8_t flag;
	int synced;
	uint8_t seq;
	int got_pkts;
	int pkt_num;
	int pkts_per_frame;
	int pkt_size;
	int frame_size;
	int last_pkt_size;
	int valid_pkts;
	unsigned int lost_pkts;
	int valid_frames;
	int variable_length;
	uint32_t last_timestamp;
	uint32_t timestamp;
	int split_bufs;
	void *lib_buf;
	void *usr_buf;
	uint8_t *raw_buf;
	void *proc_buf;
} packet_stream;

typedef struct {
	int running;

	freenect_sample_51* audio_out_ring; // TODO: implement sending user-provided data in callbacks
	int ring_reader_idx; // Index in audio_out_ring of the last sent sample
	int ring_writer_idx; // Index in audio_out_ring of the next sample we haven't received from the client yet

	uint16_t out_window;
	uint8_t out_seq;
	uint8_t out_counter_within_window;
	uint16_t out_weird_timestamp;
	uint8_t out_window_parity;

	uint16_t in_window;
	uint16_t last_seen_window[10];
	uint8_t in_counter;
	int32_t* mic_buffer[4];
	int16_t* cancelled_buffer;
	void* in_unknown;

	// TODO: timestamps
} audio_stream;
struct _freenect_device {
	freenect_context *parent;
	freenect_device *next;
	void *user_data;

	// Cameras
	fnusb_dev usb_cam;
	fnusb_isoc_stream depth_isoc;
	fnusb_isoc_stream video_isoc;

	freenect_depth_cb depth_cb;
	freenect_video_cb video_cb;
	freenect_chunk_cb depth_chunk_cb;
	freenect_chunk_cb video_chunk_cb;
	freenect_video_format video_format;
	freenect_depth_format depth_format;
	freenect_resolution video_resolution;
	freenect_resolution depth_resolution;

	int cam_inited;
	uint16_t cam_tag;

	packet_stream depth;
	packet_stream video;

	// Registration
	freenect_registration registration;

	// Audio
	fnusb_dev usb_audio;
	fnusb_isoc_stream audio_out_isoc;
	fnusb_isoc_stream audio_in_isoc;

	freenect_audio_in_cb audio_in_cb;
	freenect_audio_out_cb audio_out_cb;

	audio_stream audio;
	uint32_t audio_tag;

	// Motor
	fnusb_dev usb_motor;
	freenect_raw_tilt_state raw_state;

	int device_does_motor_control_with_audio;
	int motor_control_with_audio_enabled;
};
/*
struct freenectaudio_device;

typedef struct fnusb_dev {
	freenectaudio_device *parent; //so we can go up from the libusb userdata
	libusb_device_handle *dev;
	int device_dead; // set to 1 when the underlying libusb_device_handle vanishes (ie, Kinect was unplugged)
	int VID;
	int PID;
} fnusb_dev;

struct freenectaudio_device {
	freenect_context *parent;
	freenect_device *next;
	void *user_data;

	// Registration
	//freenect_registration registration;

	// Audio
	fnusb_dev usb_audio;
	//fnusb_isoc_stream audio_out_isoc;
	//fnusb_isoc_stream audio_in_isoc;

	//freenect_audio_in_cb audio_in_cb;
	//freenect_audio_out_cb audio_out_cb;

	//audio_stream audio;
	//uint32_t audio_tag;

	// Motor
	fnusb_dev usb_motor;
	freenect_raw_tilt_state raw_state;

	int device_does_motor_control_with_audio;
	int motor_control_with_audio_enabled;
};
*/