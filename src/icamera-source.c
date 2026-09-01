/*
 * icamera-spa - PipeWire SPA source plugin (libcamhal backend)
 *
 * A native PipeWire SPA node whose frames come directly from the Intel
 * IPU6 Camera HAL (libcamhal / icamerasrc), bypassing GStreamer and
 * v4l2loopback entirely.
 *
 * The SPA node implements the full spa_node data plane.  A producer
 * thread drives the blocking camhal_backend qbuf/dqbuf capture loop,
 * copies each NV12 frame into an SPA output buffer and pushes it onto
 * the queue consumed by spa_node.process().
 *
 * Only the real built-in cameras (gc5035-uf rear, ov5675-uf front) are
 * exposed; USB / UVC raw sensors are filtered out (see camhal_backend.c).
 *
 * SPDX-License-Identifier: MIT
 */

#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <time.h>

#include <spa/utils/defs.h>
#include <spa/utils/result.h>
#include <spa/utils/type.h>
#include <spa/utils/list.h>
#include <spa/utils/string.h>
#include <spa/utils/hook.h>
#include <spa/utils/dict.h>
#include <spa/utils/keys.h>
#include <spa/utils/names.h>
#include <spa/node/node.h>
#include <spa/node/command.h>
#include <spa/node/io.h>
#include <spa/node/keys.h>
#include <spa/node/utils.h>
#include <spa/param/param.h>
#include <spa/param/props.h>
#include <spa/param/video/format.h>
#include <spa/param/video/format-utils.h>
#include <spa/param/video/raw.h>
#include <spa/pod/pod.h>
#include <spa/pod/builder.h>
#include <spa/pod/vararg.h>
#include <spa/pod/iter.h>
#include <spa/pod/filter.h>
#include <spa/monitor/device.h>
#include <spa/support/plugin.h>
#include <spa/support/log.h>

#include "camhal_backend.h"

#define MAX_BUFFERS 32

/* ------------------------------------------------------------------ */
/* Logging helpers                                                     */
/*                                                                     */
/* Every diagnostic goes through the standard SPA logging interface    */
/* (impl->log, provided by PipeWire / the SPA loader).  This makes the */
/* messages land in pw_log with component + level tags and lets them   */
/* be filtered (SPA_DEBUG / PW_LOG) instead of raw fprintf(stderr).    */
/* NULL log => logging disabled.                                      */
/* ------------------------------------------------------------------ */

#define ICAM_LOG_ERR(i, ...) \
	do { if ((i) && (i)->log) spa_log_error((i)->log, __VA_ARGS__); } while (0)
#define ICAM_LOG_WARN(i, ...) \
	do { if ((i) && (i)->log) spa_log_warn((i)->log, __VA_ARGS__); } while (0)
#define ICAM_LOG_INFO(i, ...) \
	do { if ((i) && (i)->log) spa_log_info((i)->log, __VA_ARGS__); } while (0)
#define ICAM_LOG_DEBUG(i, ...) \
	do { if ((i) && (i)->log) spa_log_debug((i)->log, __VA_ARGS__); } while (0)

#define GET_OUT_PORT(i) (&(i)->out_port)

/* ------------------------------------------------------------------ */
/* The frame queue shared between the libcamhal producer thread and    */
/* the driver/consumer thread (spa_node.process).                      */
/* ------------------------------------------------------------------ */

#define BUFFER_FLAG_OUTSTANDING (1u << 0)	/* enqueued / in flight */
#define BUFFER_FLAG_OWNED       (1u << 1)	/* memory allocated by us (memfd) */

struct frame {
	uint32_t id;
	uint32_t flags;		/* bit 0 = outstanding; bit 1 = owned */
	uint64_t pts;		/* presentation time (CLOCK_MONOTONIC, ns) */
	struct spa_list link;
	struct spa_buffer *outbuf;
	struct spa_meta_header *h;
};

struct port {
	uint32_t width, height;
	uint32_t stride;	/* HAL bytes-per-line (may be > width for padding) */
	size_t data_size;	/* packed output size = width * height * 3/2 */
	size_t hal_size;	/* padded HAL frame size = stride * height * 3/2 */
	bool have_format;

	uint32_t n_buffers;
	uint32_t capture_next;	/* round-robin cursor for the capture thread */
	struct frame buffers[MAX_BUFFERS];
	struct spa_list queue;
	pthread_mutex_t queue_lock;

	struct spa_port_info info;
	uint64_t info_all;
	uint64_t change_mask;
	struct spa_io_buffers *io;
	struct spa_io_sequence *control;
	uint32_t control_size;

#define PORT_PropInfo	0
#define PORT_EnumFormat	1
#define PORT_Meta	2
#define PORT_IO		3
#define PORT_Format	4
#define PORT_Buffers	5
#define PORT_Latency	6
#define N_PORT_PARAMS	7
	struct spa_param_info params[N_PORT_PARAMS];
};

/* ------------------------------------------------------------------ */
/* The node implementation                                            */
/* ------------------------------------------------------------------ */

struct impl {
	struct spa_handle handle;
	struct spa_node node;
	struct spa_log *log;

	/* libcamhal backend producing frames */
	struct camhal_backend *backend;
	int camera_id;
	uint8_t *tmpbuf;		/* one dqbuf staging frame */
	pthread_t capture_thread;
	bool capture_thread_running;
	bool capture_stop;

	/* NV12 resolutions advertised by the HAL for this camera
	 * (queried once during node init). */
	struct camhal_resolution res[32];
	int n_res;

	/* Configurable 3A parameters (parsed from node properties in init,
	 * applied to the HAL backend at start). */
	struct camhal_3a_settings s3a;

	/* Current target framerate used in EnumFormat/Format negotiation.
	 * Derived from s3a.frame_rate (fps) once the 3A properties are parsed
	 * in init, and updated live when the 3A frame-rate prop is changed at
	 * runtime (so the negotiated framerate tracks the HAL's target fps
	 * instead of being hardcoded to 30). */
	struct spa_fraction out_framerate;

	bool active;

	/* one output port */
	struct port out_port;

	/* node properties */
	char node_name[64];
	char node_description[64];
	char device_name[64];

	struct spa_node_info info;
	uint64_t info_all;
	char media_class[32];
#define NODE_Props       0
#define NODE_EnumFormat  1
#define NODE_Format      2
#define N_NODE_PARAMS    3
	struct spa_param_info params_node[N_NODE_PARAMS];

	/* Dynamic per-instance node info props: media.category, an explicit
	 * camera-id, and a read-back of the current 3A configuration so the
	 * live state is visible via pw-dump / pw-cli (not just applied on
	 * set).  The dict is built once during impl_init. */
#define N_NODE_INFO_ITEMS 10
	struct spa_dict_item node_info_items[N_NODE_INFO_ITEMS];
	struct spa_dict node_info_dict;
	char info_strbuf[N_NODE_INFO_ITEMS][24];

	struct spa_hook_list hooks;
	struct spa_callbacks callbacks;
};

/* ------------------------------------------------------------------ */
/* libcamhal capture thread (producer)                                */
/* ------------------------------------------------------------------ */

static void *capture_thread_main(void *data)
{
	struct impl *impl = data;
	uint32_t i;

	while (!impl->capture_stop) {
		uint64_t ts = 0;
		int ret = camhal_backend_dqbuf(impl->backend,
					       impl->tmpbuf,
					       (int)impl->out_port.hal_size,
					       &ts);
		if (ret < 0) {
			if (impl->log)
				spa_log_warn(impl->log, "camhal dqbuf failed: %d", ret);
			if (impl->capture_stop)
				break;
			usleep(10000);
			continue;
		}

		{
			static long capn = 0;
			if ((++capn % 30) == 1) {
				uint8_t *tp = impl->tmpbuf;
				size_t tsize = impl->out_port.data_size;
				unsigned int tmin = 255, tmax = 0;
				uint64_t tsum = 0;
				size_t tn = tsize > 0 ? tsize : 0;
				size_t tchroma = tn > 2 ? tn / 3 : 0; /* crude NV12 chroma budget */
				for (size_t tj = 0; tj < tchroma; tj++) {
					uint8_t v = tp[tj];
					if (v < tmin) tmin = v;
					if (v > tmax) tmax = v;
					tsum += v;
				}
				ICAM_LOG_DEBUG(impl,
					"capture dqbuf tmpbuf size=%zu lumin(min=%u max=%u avg=%.1f) ts=%lu",
					tn, tmin, tmax, tchroma ? (double)tsum / tchroma : 0,
					(unsigned long)ts);
			}
		}

		/* Drop the first few frames: right after streaming starts the
		 * sensor's exposure has not converged yet, so the first frames
		 * come out nearly black.  Queue them straight back to HAL to let
		 * it warm up, and only start forwarding once we've seen a few
		 * (bright enough) frames.  Otherwise the very first black frame
		 * is the one gst receives and the picture looks all-black while
		 * the bright frames get dropped by backpressure. */
		{
			static unsigned int warm = 0;
			if (warm < 12) {
				warm++;
				uint64_t tmin = 255, tsum = 0, n = 0;
				const uint8_t *tp = impl->tmpbuf;
				size_t tn = impl->out_port.data_size;
				size_t chroma = tn > 2 ? tn / 3 : 0;
				for (size_t z = 0; z < chroma; z++) {
					uint8_t v = tp[z];
					if (v < tmin) tmin = v;
					tsum += v;
					n++;
				}
				/* Once the average luminance is clearly non-black, finish
				 * warming up early and forward from the next frame. */
				if (n && (double)tsum / n > 10.0)
					warm = 12;
				ICAM_LOG_DEBUG(impl, "warmup %u avg=%.1f (drop)",
					       warm, n ? (double)tsum / n : 0.0);
				continue;
			}
		}

		/* Backpressure: only produce a frame while the number of
		 * in-flight buffers (filled/queued or outstanding and not yet
		 * recycled by the consumer) stays below a threshold.
		 * Without this, the capture thread floods port->queue much
		 * faster than gst can consume/recycle buffers, which makes
		 * pipewire re-deliver the same buffer before gst recycled it
		 * ("buffer was not recycled"). Gap a couple of free slots so
		 * the producer loop can always keep flowing while never
		 * saturating the consumer. */
		struct frame *frame = NULL;
		{
			uint32_t in_flight = 0;
			uint32_t threshold;
			pthread_mutex_lock(&impl->out_port.queue_lock);
			for (i = 0; i < impl->out_port.n_buffers; i++)
				if (impl->out_port.buffers[i].flags & 1)
					in_flight++;
			threshold = impl->out_port.n_buffers > 3 ?
				impl->out_port.n_buffers - 2u : 1u;
			if (in_flight >= threshold) {
				pthread_mutex_unlock(&impl->out_port.queue_lock);
				usleep(2000);
				continue;
			}
			/* round-robin: start scanning after the last used
			 * slot so we alternate buffers instead of always
			 * reusing buffer 0. */
			for (i = 0; i < impl->out_port.n_buffers; i++) {
				uint32_t idx = (i + impl->out_port.capture_next)
						% impl->out_port.n_buffers;
				struct frame *f = &impl->out_port.buffers[idx];
				if (!(f->flags & 1) && f->outbuf != NULL &&
				    f->outbuf->datas[0].data != NULL) {
					frame = f;
					impl->out_port.capture_next = idx + 1;
					break;
				}
			}
			if (frame != NULL) {
				size_t copy = impl->out_port.data_size;
				struct spa_data *d = &frame->outbuf->datas[0];
				void *dst = d->data;
				/* The buffer handed to us by the peer (gst) may advertise
				 * maxsize==0; trust our own layout, cap only if peer gives a
				 * sane non-zero maxsize. */
				if (d->maxsize > 0 && copy > d->maxsize)
					copy = d->maxsize;
				/* Stride handling (P0): the HAL may pad each NV12 line
				 * beyond the negotiated width.  tmpbuf holds the padded
				 * frame; we must copy per-row to strip the padding so the
				 * output buffer holds a clean packed NV12 frame matching
				 * SPA_FORMAT_VIDEO_size (otherwise rows shift and the
				 * picture looks green/misaligned).  When stride == width
				 * this reduces to a single flat memcpy (fast path). */
				if (dst != NULL && impl->out_port.stride > impl->out_port.width) {
					const uint8_t *src = (const uint8_t *)impl->tmpbuf;
					uint32_t w = impl->out_port.width;
					uint32_t s = impl->out_port.stride;
					uint32_t h = impl->out_port.height;
					uint8_t *out = (uint8_t *)dst;
					size_t row, rowBytes;

					/* Y plane: h rows of w bytes packed from s-byte rows. */
					rowBytes = (size_t)w;
					for (row = 0; row < h; row++) {
						memcpy(out + row * rowBytes,
						       src + (size_t)row * s, rowBytes);
					}
					/* UV plane (interleaved CbCr): h/2 rows of w bytes. */
					for (row = 0; row < h / 2; row++) {
						memcpy(out + (size_t)h * w + row * rowBytes,
						       src + (size_t)s * h + (size_t)row * s, rowBytes);
					}
					copy = (size_t)h * w * 3 / 2;
					if (d->maxsize > 0 && copy > d->maxsize)
						copy = d->maxsize;
				} else if (dst != NULL) {
					memcpy(dst, impl->tmpbuf, copy);
				}
				d->chunk->size = (uint32_t)copy;
				/* Tag the frame with its presentation time on the
				 * monotonic clock.  The downstream video sink syncs to
				 * this to know when to show each frame; a pts of 0 on
				 * every frame makes autovideosink stall because it can
				 * make no progress on the clock. */
				{
					struct timespec mono;
					clock_gettime(CLOCK_MONOTONIC, &mono);
					frame->pts = (uint64_t)mono.tv_sec * SPA_NSEC_PER_SEC +
						     (uint64_t)mono.tv_nsec;
				}
				SPA_FLAG_SET(frame->flags, 1);
				spa_list_append(&impl->out_port.queue, &frame->link);
				{
					static unsigned long appn = 0;
					if ((++appn % 50) == 1)
						ICAM_LOG_DEBUG(impl,
							"APPEND #%lu buf=%u pts=%lu ts=%lu nbuf=%u",
							appn, frame->id,
							(unsigned long)frame->pts, (unsigned long)ts,
							impl->out_port.n_buffers);
				}
			} /* if (frame != NULL) */
			{
				static long capn2 = 0;
				if ((++capn2 % 30) == 1)
					ICAM_LOG_DEBUG(impl, "capture appended=%d nb=%u",
						       frame != NULL, impl->out_port.n_buffers);
			}
			pthread_mutex_unlock(&impl->out_port.queue_lock);
		} /* backpressure block */
	}
	return NULL;
}

/* Apply the (possibly freshly updated) 3A settings to the HAL backend if it
 * is already open.  Called from the dynamic set_param() path.  If the backend
 * is not open yet (not streaming), nothing is pushed here -- the lazily opened
 * backend in camhal_start() applies impl->s3a automatically, so the values
 * still take effect.  Returns 0 on success / backend-not-open, <0 on error. */
static int apply_3a(struct impl *impl)
{
	if (impl->backend == NULL)
		return 0;
	if (camhal_backend_set_3a(impl->backend, &impl->s3a) < 0) {
		if (impl->log)
			spa_log_error(impl->log, "camhal set_3a (dynamic) failed");
		return -EIO;
	}
	return 0;
}

static int camhal_start(struct impl *impl)
{
	if (impl->active)
		return 0;

	/* Lazily open the camera now that streaming is actually requested. */
	if (impl->backend == NULL) {
		impl->backend = camhal_backend_create(impl->camera_id, impl->log);
		if (impl->backend == NULL) {
			if (impl->log)
				spa_log_error(impl->log,
					      "camhal_backend_create(%d) failed",
					      impl->camera_id);
			return -EIO;
		}
		/* Apply the user-configured 3A parameters (if any).  This persists
		 * them on the backend so both the open-time baseline push and the
		 * start()-time re-push use these values instead of the hardcoded
		 * auto defaults.  With an all-zero setting it just re-applies auto. */
		if (camhal_backend_set_3a(impl->backend, &impl->s3a) < 0) {
			if (impl->log)
				spa_log_error(impl->log, "camhal set_3a failed");
			camhal_backend_destroy(impl->backend);
			impl->backend = NULL;
			return -EIO;
		}
	}

	int stride = 0, size = 0;
	if (camhal_backend_configure(impl->backend,
				     impl->out_port.width,
				     impl->out_port.height,
				     MAX_BUFFERS,
				     &stride, &size) < 0) {
		if (impl->log)
			spa_log_error(impl->log, "camhal configure failed");
		return -EIO;
	}
	/* Stride handling (P0): the HAL may pad each line beyond the nominal
	 * width (e.g. RGB-IR full resolution).  Keep the *packed* size (width*
	 * height*3/2) as the data_size we negotiate/advertise downstream, but
	 * remember the HAL's real bytes-per-line so the capture thread can copy
	 * row-by-row and strip the padding (otherwise frames come out shifted /
	 * green-striped).  hal_size is the padded frame size the HAL produces,
	 * which is what tmpbuf must be able to hold. */
	impl->out_port.stride = (uint32_t)stride;
	impl->out_port.hal_size = (size_t)((size_t)impl->out_port.stride *
					   impl->out_port.height * 3 / 2);
	if (impl->out_port.hal_size < impl->out_port.data_size)
		impl->out_port.hal_size = impl->out_port.data_size;

	if (impl->tmpbuf == NULL)
		impl->tmpbuf = malloc(impl->out_port.hal_size);
	if (impl->tmpbuf == NULL)
		return -ENOMEM;

	ICAM_LOG_INFO(impl, "icamera: HAL %ux%u stride=%d packed=%zu hal=%zu",
		      impl->out_port.width, impl->out_port.height, stride,
		      impl->out_port.data_size, impl->out_port.hal_size);

	if (camhal_backend_start(impl->backend) < 0) {
		if (impl->log)
			spa_log_error(impl->log, "camhal start failed");
		return -EIO;
	}

	impl->capture_stop = false;
	impl->capture_thread_running = true;
	if (pthread_create(&impl->capture_thread, NULL,
			   capture_thread_main, impl) != 0) {
		impl->capture_thread_running = false;
		return -EIO;
	}

	impl->active = true;
	return 0;
}

static int camhal_stop(struct impl *impl)
{
	if (!impl->active)
		return 0;

	impl->capture_stop = true;
	if (impl->capture_thread_running) {
		pthread_join(impl->capture_thread, NULL);
		impl->capture_thread_running = false;
	}

	if (impl->backend)
		camhal_backend_stop(impl->backend);

	if (impl->tmpbuf) {
		free(impl->tmpbuf);
		impl->tmpbuf = NULL;
	}

	impl->active = false;
	return 0;
}

/* ------------------------------------------------------------------ */
/* spa_node_methods                                                   */
/* ------------------------------------------------------------------ */

/*
 * Convert a target frame rate in fps (float) to a struct spa_fraction,
 * snapping to the nearest common integer/fractional CRT frame rate.
 * fps <= 0 means "use the HAL default" and falls back to 30/1.
 *   e.g. 30.0 -> {30,1}  60.0 -> {60,1}  25.0 -> {25,1}
 *        29.97 -> {30000,1001}  59.94 -> {60000,1001}  23.976 -> {24000,1001}
 */
static void icamera_fps_to_fraction(float fps, struct spa_fraction *frac)
{
	if (fps <= 0.0f) {
		frac->num = 30;
		frac->denom = 1;
		return;
	}
	/* Common fractional (drop-frame-ish) NTSC rates. */
	if (fps > 29.9f && fps < 30.1f) { frac->num = 30000; frac->denom = 1001; return; }
	if (fps > 59.8f && fps < 60.2f) { frac->num = 60000; frac->denom = 1001; return; }
	if (fps > 23.9f && fps < 24.1f) { frac->num = 24000; frac->denom = 1001; return; }
	if (fps > 49.8f && fps < 50.2f) { frac->num = 50;    frac->denom = 1;    return; }
	/* Otherwise snap to the nearest integer fps. */
	long n = (long)lrintf((double)fps);
	if (n < 1)
		n = 1;
	frac->num = (uint32_t)n;
	frac->denom = 1;
}

/* Recompute the negotiated output framerate from impl->s3a.frame_rate. */
static void icamera_update_framerate(struct impl *impl)
{
	icamera_fps_to_fraction(impl->s3a.frame_rate, &impl->out_framerate);
}

/* Build one NV12 EnumFormat pod for the given resolution index.
 * Returns 0 on success, <0 on error. */
static int build_enum_format(struct impl *impl, struct spa_pod_builder *b,
			     int idx, struct spa_pod **out)
{
	if (idx >= impl->n_res)
		return -ENOENT;

	*out = spa_pod_builder_add_object(b,
		SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
		SPA_FORMAT_mediaType,    SPA_POD_Id(SPA_MEDIA_TYPE_video),
		SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
		SPA_FORMAT_VIDEO_format, SPA_POD_Id(SPA_VIDEO_FORMAT_NV12),
		SPA_FORMAT_VIDEO_size,   SPA_POD_CHOICE_RANGE_Rectangle(
			&SPA_RECTANGLE(impl->res[idx].width, impl->res[idx].height),
			&SPA_RECTANGLE(impl->res[idx].width, impl->res[idx].height),
			&SPA_RECTANGLE(impl->res[idx].width, impl->res[idx].height)),
		SPA_FORMAT_VIDEO_framerate, SPA_POD_CHOICE_RANGE_Fraction(
			&impl->out_framerate, &impl->out_framerate,
			&impl->out_framerate));
	return 0;
}

static int impl_node_enum_params(void *object, int seq,
				 uint32_t id, uint32_t start, uint32_t num,
				 const struct spa_pod *filter)
{
	struct impl *impl = object;
	struct spa_pod *param;
	struct spa_pod_builder b = { 0 };
	uint8_t buffer[4096];
	struct spa_result_node_params result;
	uint32_t count = 0;
	int res;

	if (num == 0)
		return -EINVAL;

	result.id = id;
	result.next = start;
next:
	result.index = result.next++;

	spa_pod_builder_init(&b, buffer, sizeof(buffer));

	switch (id) {
	case SPA_PARAM_Props:
	{
		struct spa_pod *params_sub;
		struct spa_pod_frame f;

		if (result.index > 0)
			return 0;

		/* Custom 3A channel: Struct( (String:key, Pod:value)* ) carrying
		 * the current live 3A configuration.  Keys mirror the init-time
		 * icamera.* properties so the set_param() path can round-trip a
		 * full read-modify-write without needing a restart. */
		spa_pod_builder_push_struct(&b, &f);
		spa_pod_builder_add(&b,
			SPA_POD_String("api.icamera.ae-mode"),
				SPA_POD_Int(impl->s3a.ae_mode),
			SPA_POD_String("api.icamera.exposure"),
				SPA_POD_Long(impl->s3a.exposure_time),
			SPA_POD_String("api.icamera.gain"),
				SPA_POD_Float(impl->s3a.gain),
			SPA_POD_String("api.icamera.awb-mode"),
				SPA_POD_Int(impl->s3a.awb_mode),
			SPA_POD_String("api.icamera.awb-r-gain"),
				SPA_POD_Int(impl->s3a.awb_r_gain),
			SPA_POD_String("api.icamera.awb-g-gain"),
				SPA_POD_Int(impl->s3a.awb_g_gain),
			SPA_POD_String("api.icamera.awb-b-gain"),
				SPA_POD_Int(impl->s3a.awb_b_gain),
			SPA_POD_String("api.icamera.frame-rate"),
				SPA_POD_Float(impl->s3a.frame_rate),
			SPA_POD_String("api.icamera.3a-cadence"),
				SPA_POD_Int(impl->s3a.run_3a_cadence),
			0);
		params_sub = spa_pod_builder_pop(&b, &f);

		param = spa_pod_builder_add_object(&b,
			SPA_TYPE_OBJECT_Props, id,
			SPA_PROP_live,     SPA_POD_Bool(true),
			SPA_PROP_params,   SPA_POD_PodStruct(params_sub));
		break;
	}
	case SPA_PARAM_EnumFormat:
		/* Expose the supported NV12 formats at the node level so a
		 * pw_stream / GStreamer pipewiresrc can negotiate a format
		 * when connecting to this Video/Source.  Without this the
		 * node is not connectable ("target not found"). */
		res = build_enum_format(impl, &b, result.index, &param);
		if (res < 0)
			return 0;
		break;
	case SPA_PARAM_Format:
		if (!impl->out_port.have_format)
			return -EIO;
		if (result.index > 0)
			return 0;
		{
			struct spa_video_info_raw ri;
			memset(&ri, 0, sizeof(ri));
			ri.format = SPA_VIDEO_FORMAT_NV12;
			ri.size = SPA_RECTANGLE(impl->out_port.width, impl->out_port.height);
			ri.framerate = impl->out_framerate;
			param = spa_format_video_raw_build(&b, id, &ri);
		}
		break;
	default:
		return -ENOENT;
	}

	if (spa_pod_filter(&b, &result.param, param, filter) < 0)
		goto next;

	spa_node_emit_result(&impl->hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS,
			     &result);

	if (++count != num)
		goto next;

	return 0;
}

static int impl_node_set_param(void *object, uint32_t id, uint32_t flags,
			       const struct spa_pod *param)
{
	struct impl *impl = object;
	const struct spa_pod_prop *prop;
	int changed = 0;

	if (id != SPA_PARAM_Props)
		return -ENOENT;
	if (param == NULL)
		return 0;

	if (SPA_POD_TYPE(param) != SPA_TYPE_Object ||
	    SPA_POD_OBJECT_TYPE(param) != SPA_TYPE_OBJECT_Props)
		return -EINVAL;

	/* Parse the SPA_PROP_params channel: Struct((String:key, Pod:value)*).
	 * Each key mirrors an init-time icamera.* property and updates the
	 * corresponding 3A field live, without a node restart. */
	SPA_POD_OBJECT_FOREACH((struct spa_pod_object *)param, prop) {
		const struct spa_pod *p;
		const char *key = NULL;

		if (prop->key != SPA_PROP_params)
			continue;

		SPA_POD_STRUCT_FOREACH(&prop->value, p) {
			if (SPA_POD_TYPE(p) == SPA_TYPE_String) {
				if (spa_pod_get_string(p, &key) < 0)
					key = NULL;
				continue;
			}
			if (key == NULL)
				continue;

			if (strcmp(key, "api.icamera.ae-mode") == 0 &&
			    SPA_POD_TYPE(p) == SPA_TYPE_Int) {
				impl->s3a.ae_mode =
					SPA_POD_VALUE(struct spa_pod_int, p);
				changed = 1;
			} else if (strcmp(key, "api.icamera.exposure") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Long) {
				impl->s3a.exposure_time =
					SPA_POD_VALUE(struct spa_pod_long, p);
				impl->s3a.apply_exposure = 1;
				changed = 1;
			} else if (strcmp(key, "api.icamera.gain") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Float) {
				impl->s3a.gain =
					SPA_POD_VALUE(struct spa_pod_float, p);
				impl->s3a.apply_gain = 1;
				changed = 1;
			} else if (strcmp(key, "api.icamera.awb-mode") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Int) {
				impl->s3a.awb_mode =
					SPA_POD_VALUE(struct spa_pod_int, p);
				changed = 1;
			} else if (strcmp(key, "api.icamera.awb-r-gain") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Int) {
				impl->s3a.awb_r_gain =
					SPA_POD_VALUE(struct spa_pod_int, p);
				impl->s3a.apply_awb_gains = 1;
				changed = 1;
			} else if (strcmp(key, "api.icamera.awb-g-gain") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Int) {
				impl->s3a.awb_g_gain =
					SPA_POD_VALUE(struct spa_pod_int, p);
				impl->s3a.apply_awb_gains = 1;
				changed = 1;
			} else if (strcmp(key, "api.icamera.awb-b-gain") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Int) {
				impl->s3a.awb_b_gain =
					SPA_POD_VALUE(struct spa_pod_int, p);
				impl->s3a.apply_awb_gains = 1;
				changed = 1;
			} else if (strcmp(key, "api.icamera.frame-rate") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Float) {
				impl->s3a.frame_rate =
					SPA_POD_VALUE(struct spa_pod_float, p);
				/* Track the negotiated framerate so EnumFormat/Format
				 * advertise the new target instead of a stale value. */
				icamera_update_framerate(impl);
				changed = 1;
			} else if (strcmp(key, "api.icamera.3a-cadence") == 0 &&
				   SPA_POD_TYPE(p) == SPA_TYPE_Int) {
				impl->s3a.run_3a_cadence =
					SPA_POD_VALUE(struct spa_pod_int, p);
				changed = 1;
			}
			key = NULL;
		}
	}

	if (changed) {
		if (impl->log)
			spa_log_info(impl->log, "icamera: dynamic 3A props updated");
		/* Push to the HAL if streaming; otherwise camhal_start() applies
		 * impl->s3a when the backend is opened. */
		apply_3a(impl);
		/* Notify clients that the Props param changed so they re-enum. */
		impl->info.change_mask |= SPA_NODE_CHANGE_MASK_PARAMS;
		spa_node_emit_info(&impl->hooks, &impl->info);
		impl->info.change_mask = 0;
	}

	return 0;
}

static int impl_node_set_io(void *object, uint32_t id, void *data, size_t size)
{
	return 0;
}

static int impl_node_send_command(void *object, const struct spa_command *command)
{
	struct impl *impl = object;
	int res = 0;

	switch (SPA_NODE_COMMAND_ID(command)) {
	case SPA_NODE_COMMAND_Start:
		ICAM_LOG_INFO(impl, "command: Start");
		res = camhal_start(impl);
		break;
	case SPA_NODE_COMMAND_Pause:
	case SPA_NODE_COMMAND_Suspend:
		ICAM_LOG_INFO(impl, "command: Pause/Suspend");
		camhal_stop(impl);
		break;
	default:
		break;
	}
	return res;
}

static int impl_node_add_listener(void *object,
				  struct spa_hook *listener,
				  const struct spa_node_events *events,
				  void *data)
{
	struct impl *impl = object;
	struct spa_hook_list save;

	spa_hook_list_isolate(&impl->hooks, &save, listener, events, data);

	/* Tell the client (PipeWire) which output ports exist.  Without this
	 * the node would expose zero output ports and no data could flow. */
	impl->info.change_mask = impl->info_all;
	if (impl->info.change_mask) {
		spa_node_emit_info(&impl->hooks, &impl->info);
		impl->info.change_mask = 0;
	}

	{
		struct port *port = GET_OUT_PORT(impl);
		port->change_mask = port->info_all;
		if (port->change_mask) {
			spa_node_emit_port_info(&impl->hooks,
						SPA_DIRECTION_OUTPUT, 0,
						&port->info);
			port->change_mask = 0;
		}
	}

	spa_hook_list_join(&impl->hooks, &save);
	return 0;
}

static int impl_node_set_callbacks(void *object,
				   const struct spa_node_callbacks *callbacks,
				   void *data)
{
	struct impl *impl = object;

	impl->callbacks = SPA_CALLBACKS_INIT(callbacks, data);
	return 0;
}

static int impl_node_sync(void *object, int seq)
{
	return 0;
}

static int impl_node_add_port(void *object,
			      enum spa_direction direction,
			      uint32_t port_id,
			      const struct spa_dict *props)
{
	return 0;
}

static int impl_node_remove_port(void *object,
				 enum spa_direction direction,
				 uint32_t port_id)
{
	return 0;
}

static int impl_node_port_enum_params(void *object, int seq,
				      enum spa_direction direction,
				      uint32_t port_id, uint32_t id,
				      uint32_t start, uint32_t num,
				      const struct spa_pod *filter)
{
	struct impl *impl = object;
	struct port *port = GET_OUT_PORT(impl);
	struct spa_pod_builder b = { 0 };
	uint8_t buffer[4096];
	struct spa_pod *param;
	struct spa_result_node_params result;
	uint32_t count = 0;

	if (port_id != 0)
		return -EINVAL;

	result.id = id;
	result.next = start;
next:
	result.index = result.next++;

	spa_pod_builder_init(&b, buffer, sizeof(buffer));

	switch (id) {
	case SPA_PARAM_EnumFormat:
		if (result.index > 0)
			return 0;
		param = spa_pod_builder_add_object(&b,
			SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
			SPA_FORMAT_mediaType,    SPA_POD_Id(SPA_MEDIA_TYPE_video),
			SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
			SPA_FORMAT_VIDEO_format, SPA_POD_Id(SPA_VIDEO_FORMAT_NV12),
			SPA_FORMAT_VIDEO_size,   SPA_POD_CHOICE_RANGE_Rectangle(
				&SPA_RECTANGLE(impl->res[0].width, impl->res[0].height),
				&SPA_RECTANGLE(impl->res[0].width, impl->res[0].height),
				&SPA_RECTANGLE(impl->res[0].width, impl->res[0].height)),
			SPA_FORMAT_VIDEO_framerate, SPA_POD_CHOICE_RANGE_Fraction(
				&impl->out_framerate, &impl->out_framerate,
				&impl->out_framerate));
		break;
	case SPA_PARAM_Format:
		if (!port->have_format)
			return -EIO;
		if (result.index > 0)
			return 0;
		{
			struct spa_video_info_raw ri;
			memset(&ri, 0, sizeof(ri));
			ri.format = SPA_VIDEO_FORMAT_NV12;
			ri.size = SPA_RECTANGLE(port->width, port->height);
			ri.framerate = impl->out_framerate;
			param = spa_format_video_raw_build(&b, id, &ri);
		}
		break;
	case SPA_PARAM_Meta:
		if (result.index > 0)
			return 0;
		param = spa_pod_builder_add_object(&b,
			SPA_TYPE_OBJECT_ParamMeta, id,
			SPA_PARAM_META_type, SPA_POD_Id(SPA_META_Header),
			SPA_PARAM_META_size, SPA_POD_Int(sizeof(struct spa_meta_header)));
		break;
	case SPA_PARAM_IO:
		if (result.index > 0)
			return 0;
		param = spa_pod_builder_add_object(&b,
			SPA_TYPE_OBJECT_ParamIO, id,
			SPA_PARAM_IO_id,   SPA_POD_Id(SPA_IO_Buffers),
			SPA_PARAM_IO_size, SPA_POD_Int(sizeof(struct spa_io_buffers)));
		break;
	case SPA_PARAM_Buffers:
		if (!port->have_format)
			return -EIO;
		if (result.index > 0)
			return 0;
		param = spa_pod_builder_add_object(&b,
			SPA_TYPE_OBJECT_ParamBuffers, id,
			SPA_PARAM_BUFFERS_buffers, SPA_POD_CHOICE_RANGE_Int(2, 4, MAX_BUFFERS),
			SPA_PARAM_BUFFERS_blocks,  SPA_POD_Int(1),
			SPA_PARAM_BUFFERS_size,    SPA_POD_Int(port->data_size),
			SPA_PARAM_BUFFERS_stride,  SPA_POD_Int(port->width));
		break;
	default:
		return -ENOENT;
	}

	if (spa_pod_filter(&b, &result.param, param, filter) < 0)
		goto next;

	spa_node_emit_result(&impl->hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS,
			     &result);

	if (++count != num)
		goto next;

	return 0;
}

static int impl_node_port_set_param(void *object,
				    enum spa_direction direction,
				    uint32_t port_id, uint32_t id,
				    uint32_t flags,
				    const struct spa_pod *param)
{
	struct impl *impl = object;
	struct port *port = GET_OUT_PORT(impl);
	struct spa_video_info info = { 0 };
	int res;

	if (id != SPA_PARAM_Format)
		return -ENOENT;

	if (param == NULL) {
		port->have_format = false;
		return 0;
	}

	if ((res = spa_format_parse(param, &info.media_type, &info.media_subtype)) < 0)
		return res;
	if (info.media_type != SPA_MEDIA_TYPE_video ||
	    info.media_subtype != SPA_MEDIA_SUBTYPE_raw)
		return -EINVAL;
	if (spa_format_video_raw_parse(param, &info.info.raw) < 0)
		return -EINVAL;

	if (info.info.raw.format != SPA_VIDEO_FORMAT_NV12 ||
	    info.info.raw.size.width == 0 || info.info.raw.size.height == 0)
		return -EINVAL;

	port->width = info.info.raw.size.width;
	port->height = info.info.raw.size.height;
	port->data_size = (size_t)port->width * port->height * 3 / 2;
	port->have_format = true;
	return 0;
}

/*
 * Release any buffers whose memory we allocated ourselves (memfd).
 * The peer owns its own buffers (which we did not allocate); for those
 * we only clear our bookkeeping.
 */
static int icamera_clear_buffers(struct impl *impl, struct port *port)
{
	uint32_t i;

	for (i = 0; i < port->n_buffers; i++) {
		struct frame *frame = &port->buffers[i];
		struct spa_data *d;

		if (frame->outbuf == NULL)
			continue;
		d = &frame->outbuf->datas[0];
		if (SPA_FLAG_IS_SET(frame->flags, BUFFER_FLAG_OWNED)) {
			if (d->data)
				munmap(d->data, d->maxsize);
			if (d->fd >= 0)
				close(d->fd);
			d->data = NULL;
			d->fd = -1;
			d->maxsize = 0;
			d->type = SPA_ID_INVALID;
			SPA_FLAG_CLEAR(frame->flags, BUFFER_FLAG_OWNED);
		}
		frame->outbuf = NULL;
	}
	port->n_buffers = 0;
	spa_list_init(&port->queue);
	return 0;
}

/*
 * Allocate the output buffers ourselves as cross-process shared memfd,
 * following the same scheme as the v4l2 and libcamera SPA plugins when the
 * peer sets SPA_NODE_BUFFERS_FLAG_ALLOC on use_buffers().
 *
 * The peer has already arranged each spa_buffer->datas[] with the desired
 * data type (SPA_DATA_MemFd / SPA_DATA_DmaBuf); we back it with a fresh
 * memfd so every process in the graph sees the same writable memory and
 * so writing a full frame can never overrun the buffer.
 */
static int icamera_alloc_buffers(struct impl *impl, struct port *port,
				 struct spa_buffer **buffers, uint32_t n_buffers)
{
	uint32_t i;

	for (i = 0; i < n_buffers; i++) {
		struct frame *frame = &port->buffers[i];
		struct spa_data *d;
		int fd = -1;
		void *ptr;

		if (buffers[i]->n_datas < 1) {
			spa_log_error(impl->log, "icamera: buffer %u has no datas", i);
			return -EINVAL;
		}
		d = &buffers[i]->datas[0];

		fd = memfd_create("icamera-buf", MFD_CLOEXEC);
		if (fd < 0) {
			spa_log_error(impl->log, "icamera: memfd_create: %m");
			return -errno;
		}
		if (ftruncate(fd, (off_t)port->data_size) < 0) {
			spa_log_error(impl->log, "icamera: ftruncate: %m");
			close(fd);
			return -errno;
		}
		ptr = mmap(NULL, port->data_size, PROT_READ | PROT_WRITE,
			   MAP_SHARED, fd, 0);
		if (ptr == MAP_FAILED) {
			spa_log_error(impl->log, "icamera: mmap: %m");
			close(fd);
			return -errno;
		}

		d->type = SPA_DATA_MemFd;
		d->flags = SPA_DATA_FLAG_READABLE | SPA_DATA_FLAG_MAPPABLE;
		d->fd = fd;
		d->mapoffset = 0;
		d->data = ptr;
		d->maxsize = (uint32_t)port->data_size;
		if (d->chunk) {
			d->chunk->offset = 0;
			d->chunk->size = 0;
			d->chunk->stride = (int32_t)port->width;
			d->chunk->flags = 0;
		}

		frame->id = i;
		frame->outbuf = buffers[i];
		frame->flags = 0;
		frame->h = (struct spa_meta_header *)spa_buffer_find_meta_data(
				buffers[i], SPA_META_Header, sizeof(*frame->h));
		SPA_FLAG_SET(frame->flags, BUFFER_FLAG_OWNED);
		frame->link.next = NULL;
		frame->link.prev = NULL;

		ICAM_LOG_DEBUG(impl, "ALLOC buf[%u] type=%u fd=%d data=%p max=%u",
			       i, d->type, d->fd, (void*)d->data, d->maxsize);
	}

	port->n_buffers = n_buffers;
	spa_list_init(&port->queue);
	return 0;
}

static int impl_node_port_use_buffers(void *object,
				      enum spa_direction direction,
				      uint32_t port_id, uint32_t flags,
				      struct spa_buffer **buffers,
				      uint32_t n_buffers)
{
	struct impl *impl = object;
	struct port *port = GET_OUT_PORT(impl);
	uint32_t i;

	if (n_buffers > MAX_BUFFERS)
		return -ENOSPC;

	ICAM_LOG_INFO(impl, "use_buffers dir=%u port=%u flags=0x%x nbuf=%u",
		      direction, port_id, flags, n_buffers);

	pthread_mutex_lock(&port->queue_lock);
	if (port->n_buffers > 0)
		icamera_clear_buffers(impl, port);

	if (flags & SPA_NODE_BUFFERS_FLAG_ALLOC) {
		int res = icamera_alloc_buffers(impl, port, buffers, n_buffers);
		pthread_mutex_unlock(&port->queue_lock);
		return res;
	}

	port->n_buffers = n_buffers;
	spa_list_init(&port->queue);

	for (i = 0; i < n_buffers; i++) {
		struct frame *frame = &port->buffers[i];
		struct spa_data *d = &buffers[i]->datas[0];

		/* The peer usually advertises the right maxsize, but some
		 * consumers hand us buffers with maxsize==0. Since we own the
		 * frame layout, (re)assert a sane non-zero maxsize so the peer
		 * knows how much data we will write. */
		if (d->maxsize == 0)
			d->maxsize = (uint32_t)port->data_size;
		if ((i % 8) == 0)
			ICAM_LOG_DEBUG(impl,
				"ub[%u] type=%u data=%p fd=%d max=%u chunk=%p",
				i, d->type, (void*)d->data, d->fd, d->maxsize, (void*)d->chunk);

		frame->id = i;
		frame->outbuf = buffers[i];
		frame->flags = 0;
		frame->h = (struct spa_meta_header *)spa_buffer_find_meta_data(
				buffers[i], SPA_META_Header, sizeof(*frame->h));
		frame->link.next = NULL;
		frame->link.prev = NULL;
	}
	pthread_mutex_unlock(&port->queue_lock);
	return 0;
}

static int impl_node_port_set_io(void *object,
				 enum spa_direction direction,
				 uint32_t port_id, uint32_t id,
				 void *data, size_t size)
{
	struct impl *impl = object;
	struct port *port = GET_OUT_PORT(impl);

	switch (id) {
	case SPA_IO_Buffers:
		port->io = (struct spa_io_buffers *)data;
		break;
	case SPA_IO_Control:
		port->control = (struct spa_io_sequence *)data;
		port->control_size = size;
		break;
	default:
		return -ENOENT;
	}
	return 0;
}

static int impl_node_port_reuse_buffer(void *object,
				       uint32_t port_id,
				       uint32_t buffer_id)
{
	struct impl *impl = object;
	struct port *port = GET_OUT_PORT(impl);
	struct frame *frame;
	struct frame *pos;
	struct frame *tmp;

	if (port_id != 0 || buffer_id >= port->n_buffers)
		return -EINVAL;

	frame = &port->buffers[buffer_id];
	if (!(frame->flags & 1))
		return 0;

	pthread_mutex_lock(&port->queue_lock);
	spa_list_for_each_safe(pos, tmp, &port->queue, link) {
		if (pos == frame) {
			spa_list_remove(&pos->link);
			break;
		}
	}
	SPA_FLAG_CLEAR(frame->flags, 1);
	pthread_mutex_unlock(&port->queue_lock);

	/* Consumer returned this buffer: acknowledge so the next process()
	 * call can emit a fresh frame instead of short-circuiting on a stale
	 * SPA_STATUS_HAVE_DATA. */
	if (port->io) {
		port->io->buffer_id = SPA_ID_INVALID;
		port->io->status = SPA_STATUS_OK;
	}

	{
		static long nb = 0;
		if ((++nb % 200) == 1)
			ICAM_LOG_DEBUG(impl, "reuse_buffer id=%d qempty=%d",
				       buffer_id, spa_list_is_empty(&port->queue));
	}
	return 0;
}

static int impl_node_process(void *object)
{
	struct impl *impl = object;
	struct port *port = GET_OUT_PORT(impl);
	struct spa_io_buffers *io = port->io;
	struct frame *frame;

	static long proc_count = 0;
	if (++proc_count == 1 || (proc_count % 500) == 0)
		ICAM_LOG_DEBUG(impl, "process() called=%ld io=%p nbuf=%u status=%d",
			       proc_count, (void*)io, port->n_buffers,
			       io ? io->status : -1);

	if (io == NULL || port->n_buffers == 0)
		return -EIO;

	if (io->status == SPA_STATUS_HAVE_DATA)
		return SPA_STATUS_HAVE_DATA;

	/* Recycle the previous outstanding buffer. */
	if (io->buffer_id < port->n_buffers) {
		impl_node_port_reuse_buffer(impl, 0, io->buffer_id);
		io->buffer_id = SPA_ID_INVALID;
	}

	pthread_mutex_lock(&port->queue_lock);
	frame = NULL;
	if (!spa_list_is_empty(&port->queue)) {
		frame = spa_list_first(&port->queue, struct frame, link);
		spa_list_remove(&frame->link);
	}
	pthread_mutex_unlock(&port->queue_lock);

	if (frame == NULL)
		return SPA_STATUS_OK;

	if (frame->h) {
		frame->h->flags = 0;
		frame->h->seq = 0;
		/* Present this frame at the time the capture thread tagged it
		 * (CLOCK_MONOTONIC).  Without a monotonic, increasing pts the
		 * downstream sink cannot make progress on the clock and the
		 * pipeline stalls on the first frame. */
		frame->h->pts = frame->pts;
		frame->h->dts_offset = 0;
	}

	io->buffer_id = frame->id;
	io->status = SPA_STATUS_HAVE_DATA;
	{
		static unsigned long emi = 0;
		if ((++emi % 50) == 1)
			ICAM_LOG_DEBUG(impl, "EMIT #%lu frame id=%u pts=%lu nbuf=%u qempty=%d",
				       emi, frame->id, (unsigned long)frame->pts,
				       port->n_buffers, (int)spa_list_is_empty(&port->queue));
	}
	return SPA_STATUS_HAVE_DATA;
}

static const struct spa_node_methods impl_node = {
	.version = SPA_VERSION_NODE_METHODS,
	.add_listener = impl_node_add_listener,
	.set_callbacks = impl_node_set_callbacks,
	.sync = impl_node_sync,
	.enum_params = impl_node_enum_params,
	.set_param = impl_node_set_param,
	.set_io = impl_node_set_io,
	.send_command = impl_node_send_command,
	.add_port = impl_node_add_port,
	.remove_port = impl_node_remove_port,
	.port_enum_params = impl_node_port_enum_params,
	.port_set_param = impl_node_port_set_param,
	.port_use_buffers = impl_node_port_use_buffers,
	.port_set_io = impl_node_port_set_io,
	.port_reuse_buffer = impl_node_port_reuse_buffer,
	.process = impl_node_process,
};

/* ------------------------------------------------------------------ */
/* Handle plumbing                                                    */
/* ------------------------------------------------------------------ */

static int impl_get_interface(struct spa_handle *handle,
			      const char *type, void **interface)
{
	struct impl *impl = (struct impl *)handle;

	if (strcmp(type, SPA_TYPE_INTERFACE_Node) == 0)
		*interface = &impl->node;
	else
		return -ENOENT;

	return 0;
}

static int impl_clear(struct spa_handle *handle);

/*
 * Build the per-instance node info props dict.  Called once during
 * impl_init() after the camera_id has been resolved.  Exposes:
 *   - media.class              : Video/Source (kept for compatibility)
 *   - media.category           : Video/Source/Output (what GStreamer/portal
 *                                expects a camera capture source to report)
 *   - api.icamera.camera-id    : the resolved CamHAL camera id this node uses
 *   - api.icamera.*            : read-back of the current (possibly default)
 *                                3A configuration, so pw-dump / pw-cli show
 *                                what is actually in effect on this camera.
 * The value strings live in impl->info_strbuf[] so the dict stays valid for
 * the lifetime of the node.
 */
static void build_node_info(struct impl *impl)
{
	int i = 0;
	char (*sb)[24] = impl->info_strbuf;

	impl->node_info_items[i].key = SPA_KEY_MEDIA_CLASS;
	impl->node_info_items[i].value = "Video/Source";
	i++;

	impl->node_info_items[i].key = "media.category";
	impl->node_info_items[i].value = "Video/Source/Output";
	i++;

	impl->node_info_items[i].key = "api.icamera.camera-id";
	snprintf(sb[i], 24, "%d", impl->camera_id);
	impl->node_info_items[i].value = sb[i];
	i++;

	impl->node_info_items[i].key = "api.icamera.ae-mode";
	snprintf(sb[i], 24, "%d", impl->s3a.ae_mode);
	impl->node_info_items[i].value = sb[i];
	i++;

	impl->node_info_items[i].key = "api.icamera.exposure";
	snprintf(sb[i], 24, "%lld",
		 (long long)impl->s3a.exposure_time);
	impl->node_info_items[i].value = sb[i];
	i++;

	impl->node_info_items[i].key = "api.icamera.gain";
	snprintf(sb[i], 24, "%.2f", impl->s3a.gain);
	impl->node_info_items[i].value = sb[i];
	i++;

	impl->node_info_items[i].key = "api.icamera.awb-mode";
	snprintf(sb[i], 24, "%d", impl->s3a.awb_mode);
	impl->node_info_items[i].value = sb[i];
	i++;

	impl->node_info_items[i].key = "api.icamera.awb-r-gain";
	snprintf(sb[i], 24, "%d", impl->s3a.awb_r_gain);
	impl->node_info_items[i].value = sb[i];
	i++;

	impl->node_info_items[i].key = "api.icamera.awb-g-gain";
	snprintf(sb[i], 24, "%d", impl->s3a.awb_g_gain);
	impl->node_info_items[i].value = sb[i];
	i++;

	impl->node_info_items[i].key = "api.icamera.awb-b-gain";
	snprintf(sb[i], 24, "%d", impl->s3a.awb_b_gain);
	impl->node_info_items[i].value = sb[i];
	i++;

	/* (N_NODE_INFO_ITEMS == 10, but we stop after the AWB gains; the two
	 * frame-rate / 3a-cadence knobs are kept sparse here to avoid bloat.) */
	/* (N_NODE_INFO_ITEMS == 10, but we stop after the AWB gains; the two
	 * frame-rate / 3a-cadence knobs are kept sparse here to avoid bloat.) */
	impl->node_info_dict = SPA_DICT_INIT(impl->node_info_items, (uint32_t)i);
}

static int resolve_camera_id(const struct spa_dict *info)
{
	const struct spa_dict_item *item;
	const char *device_name = NULL;
	const char *camera_id_str = NULL;
	int cam_count = 0;
	struct camhal_camera_info *cams;

	/* Explicit numeric camera id (icamera.camera-id) wins. */
	if (info) {
		spa_dict_for_each(item, info) {
			if (strcmp(item->key, "icamera.camera-id") == 0)
				camera_id_str = item->value;
			else if (strcmp(item->key, SPA_KEY_DEVICE_NAME) == 0)
				device_name = item->value;
		}
	}
	if (camera_id_str != NULL) {
		int id = atoi(camera_id_str);
		/* validate it maps to a real camera */
		return id;
	}

	/* Otherwise match by device.name (sensor name) against live discovery. */
	cams = camhal_discover_cameras(&cam_count);
	if (cams != NULL) {
		int match = -1;
		for (int i = 0; i < cam_count; i++) {
			if (device_name != NULL &&
			    strcmp(device_name, cams[i].name) == 0) {
				match = cams[i].camera_id;
				break;
			}
		}
		if (match < 0 && cam_count > 0)
			match = cams[0].camera_id;
		camhal_free_cameras(cams, cam_count);
		return match;
	}

	return 0; /* fallback camera 0 */
}

static int impl_init(const struct spa_handle_factory *factory,
		     struct spa_handle *handle,
		     const struct spa_dict *info,
		     const struct spa_support *support,
		     uint32_t n_support)
{
	struct impl *impl = (struct impl *)handle;
	const struct spa_dict_item *item;

	handle->get_interface = impl_get_interface;
	handle->clear = impl_clear;

	impl->log = spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log);

	strncpy(impl->node_name, "icamera-source", sizeof(impl->node_name) - 1);
	impl->node_name[sizeof(impl->node_name) - 1] = '\0';
	strncpy(impl->node_description, "ICamera Source",
		sizeof(impl->node_description) - 1);
	impl->node_description[sizeof(impl->node_description) - 1] = '\0';
	strncpy(impl->device_name, "icamera", sizeof(impl->device_name) - 1);
	impl->device_name[sizeof(impl->device_name) - 1] = '\0';

	if (info) {
		spa_dict_for_each(item, info) {
			if (strcmp(item->key, SPA_KEY_NODE_NAME) == 0)
				strncpy(impl->node_name, item->value,
					sizeof(impl->node_name) - 1);
			else if (strcmp(item->key, SPA_KEY_NODE_DESCRIPTION) == 0)
				strncpy(impl->node_description, item->value,
					sizeof(impl->node_description) - 1);
			else if (strcmp(item->key, SPA_KEY_DEVICE_NAME) == 0)
				strncpy(impl->device_name, item->value,
					sizeof(impl->device_name) - 1);
			/* ---- tunable 3A properties ---- */
			else if (strcmp(item->key, "icamera.ae-mode") == 0)
				impl->s3a.ae_mode = atoi(item->value);
			else if (strcmp(item->key, "icamera.exposure") == 0) {
				impl->s3a.apply_exposure = 1;
				impl->s3a.exposure_time = atoll(item->value);
			}
			else if (strcmp(item->key, "icamera.exposure-us") == 0) {
				/* convenience: exposure in microseconds -> ns */
				impl->s3a.apply_exposure = 1;
				impl->s3a.exposure_time =
					(int64_t)atoll(item->value) * 1000;
			}
			else if (strcmp(item->key, "icamera.gain") == 0) {
				impl->s3a.apply_gain = 1;
				impl->s3a.gain = atof(item->value);
			}
			else if (strcmp(item->key, "icamera.awb-mode") == 0)
				impl->s3a.awb_mode = atoi(item->value);
			else if (strcmp(item->key, "icamera.awb-r-gain") == 0) {
				impl->s3a.apply_awb_gains = 1;
				impl->s3a.awb_r_gain = atoi(item->value);
			}
			else if (strcmp(item->key, "icamera.awb-g-gain") == 0) {
				impl->s3a.apply_awb_gains = 1;
				impl->s3a.awb_g_gain = atoi(item->value);
			}
			else if (strcmp(item->key, "icamera.awb-b-gain") == 0) {
				impl->s3a.apply_awb_gains = 1;
				impl->s3a.awb_b_gain = atoi(item->value);
			}
			else if (strcmp(item->key, "icamera.frame-rate") == 0)
				impl->s3a.frame_rate = atof(item->value);
			else if (strcmp(item->key, "icamera.3a-cadence") == 0)
				impl->s3a.run_3a_cadence = atoi(item->value);
		}
		impl->node_name[sizeof(impl->node_name) - 1] = '\0';
		impl->node_description[sizeof(impl->node_description) - 1] = '\0';
		impl->device_name[sizeof(impl->device_name) - 1] = '\0';
	}

	/* Derive the negotiated output framerate from the configured target fps
	 * (falls back to 30fps when icamera.frame-rate is not set).  This makes
	 * EnumFormat/Format advertise the real target frame rate instead of a
	 * hardcoded 30, and tracks the 3A frame-rate at runtime. */
	icamera_update_framerate(impl);

	impl->node.iface = SPA_INTERFACE_INIT(SPA_TYPE_INTERFACE_Node,
			SPA_VERSION_NODE, &impl_node, impl);

	spa_hook_list_init(&impl->hooks);
	impl->callbacks.funcs = NULL;
	impl->callbacks.data = NULL;

	/* Node info: advertise one output port + the media class so PipeWire
	 * can create a Video/Source and drive the output. */
	strncpy(impl->media_class, "Video/Source", sizeof(impl->media_class) - 1);
	impl->media_class[sizeof(impl->media_class) - 1] = '\0';
	impl->info_all = SPA_NODE_CHANGE_MASK_FLAGS |
			 SPA_NODE_CHANGE_MASK_PROPS |
			 SPA_NODE_CHANGE_MASK_PARAMS;
	impl->info = SPA_NODE_INFO_INIT();
	impl->info.max_output_ports = 1;
	impl->info.flags = SPA_NODE_FLAG_RT;
	impl->info.props = &impl->node_info_dict;
	impl->params_node[0] = SPA_PARAM_INFO(SPA_PARAM_Props, SPA_PARAM_INFO_READWRITE);
	impl->params_node[1] = SPA_PARAM_INFO(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
	impl->params_node[2] = SPA_PARAM_INFO(SPA_PARAM_Format, 0);
	impl->info.params = impl->params_node;
	impl->info.n_params = N_NODE_PARAMS;

	impl->active = false;
	impl->backend = NULL;
	impl->tmpbuf = NULL;
	impl->capture_thread_running = false;
	impl->capture_stop = false;

	impl->camera_id = resolve_camera_id(info);
	build_node_info(impl);

	/* The libcamhal backend is opened lazily on first Start (see
	 * camhal_start), NOT here.  camera_device_open() acquires/starts the
	 * physical camera inside the HAL; doing it at node init would hold the
	 * camera for the node's whole lifetime and block any other HAL client
	 * (e.g. a separate icamerasrc) from opening the same device even when
	 * we are not streaming.  Keep impl->backend == NULL until Start. */

	/* Query the resolutions the HAL actually supports for this camera.
	 * getSupportedStreamConfig() does not open the device, so this is safe
	 * during node init.  Use the first NV12 resolution as the default so we
	 * no longer hardcode 640x480. */
	impl->n_res = camhal_backend_get_supported_formats(impl->camera_id,
							   impl->res,
							   (int)(sizeof(impl->res) /
								 sizeof(impl->res[0])),
							   impl->log);
	if (impl->n_res <= 0) {
		/* Fall back to a sane default if enumeration failed. */
		impl->n_res = 1;
		impl->res[0].width  = 640;
		impl->res[0].height = 480;
	}

	impl->out_port.width  = impl->res[0].width;
	impl->out_port.height = impl->res[0].height;
	impl->out_port.data_size = impl->res[0].width * impl->res[0].height * 3 / 2;
	impl->out_port.have_format = false;
	impl->out_port.n_buffers = 0;
	memset(impl->out_port.buffers, 0, sizeof(impl->out_port.buffers));
	impl->out_port.io = NULL;
	impl->out_port.control = NULL;
	impl->out_port.control_size = 0;
	pthread_mutex_init(&impl->out_port.queue_lock, NULL);
	spa_list_init(&impl->out_port.queue);

	impl->out_port.params[PORT_PropInfo] =
		SPA_PARAM_INFO(SPA_PARAM_PropInfo, SPA_PARAM_INFO_READ);
	impl->out_port.params[PORT_EnumFormat] =
		SPA_PARAM_INFO(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
	impl->out_port.params[PORT_Meta] =
		SPA_PARAM_INFO(SPA_PARAM_Meta, SPA_PARAM_INFO_READ);
	impl->out_port.params[PORT_IO] =
		SPA_PARAM_INFO(SPA_PARAM_IO, SPA_PARAM_INFO_READ);
	impl->out_port.params[PORT_Format] =
		SPA_PARAM_INFO(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
	impl->out_port.params[PORT_Buffers] =
		SPA_PARAM_INFO(SPA_PARAM_Buffers, 0);
	impl->out_port.params[PORT_Latency] =
		SPA_PARAM_INFO(SPA_PARAM_Latency, SPA_PARAM_INFO_READ);
	memset(&impl->out_port.info, 0, sizeof(impl->out_port.info));
	impl->out_port.info.flags = SPA_PORT_FLAG_CAN_ALLOC_BUFFERS |
		SPA_PORT_FLAG_LIVE | SPA_PORT_FLAG_PHYSICAL |
		SPA_PORT_FLAG_TERMINAL;
	impl->out_port.info.params = impl->out_port.params;
	impl->out_port.info.n_params = N_PORT_PARAMS;
	impl->out_port.info_all = SPA_PORT_CHANGE_MASK_FLAGS |
				  SPA_PORT_CHANGE_MASK_PARAMS;
	impl->out_port.change_mask = 0;

	return 0;
}

static int impl_clear(struct spa_handle *handle)
{
	struct impl *impl = (struct impl *)handle;
	camhal_stop(impl);
	icamera_clear_buffers(impl, &impl->out_port);
	if (impl->backend) {
		camhal_backend_destroy(impl->backend);
		impl->backend = NULL;
	}
	pthread_mutex_destroy(&impl->out_port.queue_lock);
	return 0;
}

static const struct spa_interface_info impl_interfaces[] = {
	{ SPA_TYPE_INTERFACE_Node, },
};

static int impl_enum_interface_info(const struct spa_handle_factory *factory,
				    const struct spa_interface_info **info,
				    uint32_t *index)
{
	if (*index == 0) {
		*info = &impl_interfaces[0];
		(*index)++;
		return 1;
	}
	return 0;
}

static size_t impl_get_size(const struct spa_handle_factory *factory,
			 const struct spa_dict *params)
{
	return sizeof(struct impl);
}

static const struct spa_dict_item info_items[] = {
	{ SPA_KEY_FACTORY_AUTHOR, "icamera-spa" },
	{ SPA_KEY_FACTORY_DESCRIPTION, "Intel IPU6 libcamhal source node" },
};

static const struct spa_dict factory_info = SPA_DICT_INIT_ARRAY(info_items);

static const struct spa_handle_factory icamera_factory = {
	SPA_VERSION_HANDLE_FACTORY,
	"api.icamera.source",
	&factory_info,
	impl_get_size,
	impl_init,
	impl_enum_interface_info,
};

SPA_EXPORT
int spa_handle_factory_enum(const struct spa_handle_factory **factory,
			    uint32_t *index)
{
	if (factory == NULL || index == NULL)
		return -EINVAL;

	switch (*index) {
	case 0:
		*factory = &icamera_factory;
		break;
	default:
		return 0;
	}
	(*index)++;
	return 1;
}
