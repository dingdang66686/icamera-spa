/*
 * camhal_backend.cpp - C++ bridge to Intel IPU6 libcamhal
 *
 * Implements the C interface declared in camhal_backend.h using the
 * icamera:: C++ API shipped with libcamhal (see ICamera.h).  The PipeWire
 * SPA plugin uses this backend to pull NV12 frames directly from the
 * camera sensor, replacing the former GStreamer/icamerasrc producer.
 *
 * Reference: gst-camera (icamerasrc) interacts with libcamhal with the
 * following sequence, which we reproduce here:
 *   1. camera_hal_init()                 (ref-counted global singleton)
 *   2. camera_device_open(camera_id)
 *   3. camera_device_config_streams(&stream_list)
 *   4. for each buffer: camera_device_allocate_memory(&buf) + qbuf
 *   5. camera_device_start()
 *   6. loop: camera_stream_dqbuf() -> use -> camera_stream_qbuf()
 *   7. camera_device_stop(); camera_device_close(); camera_hal_deinit()
 *
 * SPDX-License-Identifier: MIT
 */

#include "camhal_backend.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <new>

#include <spa/support/log.h>

#include <libcamhal/api/ICamera.h>
#include <libcamhal/api/Parameters.h>
#include <libcamhal/linux/videodev2.h>

using namespace icamera;

/* ------------------------------------------------------------------ */
/* Logging helpers                                                     */
/*                                                                     */
/* All diagnostics go through the standard SPA logging interface the   */
/* SPA node provided at create time, so they land in the same PipeWire */
/* pw_log stream (filterable by level / component) instead of raw      */
/* fprintf(stderr).  NULL log => logging disabled.  Messages are       */
/* prefixed "camhal" (the SPA logger already tags level + component).  */
/* ------------------------------------------------------------------ */

#define CAM_LOG_ERR(b, ...) \
	do { if ((b) && (b)->log) spa_log_error((b)->log, __VA_ARGS__); } while (0)
#define CAM_LOG_WARN(b, ...) \
	do { if ((b) && (b)->log) spa_log_warn((b)->log, __VA_ARGS__); } while (0)
#define CAM_LOG_INFO(b, ...) \
	do { if ((b) && (b)->log) spa_log_info((b)->log, __VA_ARGS__); } while (0)
#define CAM_LOG_DEBUG(b, ...) \
	do { if ((b) && (b)->log) spa_log_debug((b)->log, __VA_ARGS__); } while (0)

/* ------------------------------------------------------------------ */
/* Per-format packed frame-size helper.                                */
/*                                                                     */
/* Computes the nominal byte size of a packed frame for a V4L2 pixel   */
/* fourcc (no line padding).  Used as a fallback when the HAL does not */
/* give us an exact stream size (e.g. a hand-built fallback stream).   */
/* ------------------------------------------------------------------ */
static size_t camhal_packed_size(uint32_t fourcc, uint32_t w, uint32_t h)
{
	size_t px = (size_t)w * h;
	switch (fourcc) {
	case V4L2_PIX_FMT_NV12:   /* Y plane + interleaved UV */
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YUV420: /* I420: Y + U + V */
	case V4L2_PIX_FMT_YVU420: /* YV12: Y + V + U */
		return px * 3 / 2;
	case V4L2_PIX_FMT_YUYV:   /* 2 bytes / pixel */
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUV422P:
		return px * 2;
	case V4L2_PIX_FMT_RGB24:  /* 3 bytes / pixel */
	case V4L2_PIX_FMT_BGR24:
		return px * 3;
	case V4L2_PIX_FMT_GREY:   /* 1 byte / pixel */
		return px;
	default:
		/* Unknown format: fall back to 3 bytes / pixel so callers always
		 * get a non-zero, conservative size. */
		return px * 3;
	}
}

/* ------------------------------------------------------------------ */
/* Global, ref-counted camera_hal_init/deinit so multiple SPA node      */
/* instances (one per real camera) can share the singleton safely.      */
/* ------------------------------------------------------------------ */

static pthread_mutex_t g_hal_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_hal_refs = 0;

static int hal_ref(void)
{
	pthread_mutex_lock(&g_hal_lock);
	int ret = camera_hal_init();
	if (ret == 0)
		g_hal_refs++;
	pthread_mutex_unlock(&g_hal_lock);
	return ret;
}

static void hal_unref(void)
{
	pthread_mutex_lock(&g_hal_lock);
	if (g_hal_refs > 0)
		g_hal_refs--;
	if (g_hal_refs == 0)
		camera_hal_deinit();
	pthread_mutex_unlock(&g_hal_lock);
}

/* ------------------------------------------------------------------ */
/* Discovery (only real built-in cameras, exclude USB / UVC)           */
/* ------------------------------------------------------------------ */

static bool name_is_usb(const char *name)
{
	if (!name)
		return false;
	if (strstr(name, "usb") || strstr(name, "USB") ||
	    strstr(name, "uvc") || strstr(name, "UVC"))
		return true;
	return false;
}

struct camhal_camera_info *camhal_discover_cameras(int *out_count)
{
	struct camhal_camera_info *cams = NULL;
	int count = 0;
	int total = 0;

	if (!out_count)
		return NULL;
	*out_count = 0;

	total = get_number_of_cameras();
	if (total <= 0)
		return NULL;

	cams = (struct camhal_camera_info *)calloc(total, sizeof(*cams));
	if (!cams)
		return NULL;

	for (int id = 0; id < total; id++) {
		camera_info_t info;
		memset(&info, 0, sizeof(info));
		if (get_camera_info(id, info) < 0)
			continue;
		/* Only real built-in sensors (facing valid, not USB). */
		if (name_is_usb(info.name))
			continue;
		if (info.name == NULL || info.name[0] == '\0')
			continue;

		struct camhal_camera_info *c = &cams[count];
		c->camera_id = id;
		c->facing = info.facing;
		snprintf(c->name, sizeof(c->name), "%s", info.name);
		snprintf(c->description, sizeof(c->description), "%s",
			 info.description ? info.description : info.name);
		count++;
	}

	if (count == 0) {
		free(cams);
		return NULL;
	}
	*out_count = count;
	return cams;
}

void camhal_free_cameras(struct camhal_camera_info *cams, int count)
{
	(void)count;
	free(cams);
}

/* ------------------------------------------------------------------ */
/* Backend object                                                      */
/* ------------------------------------------------------------------ */

	struct camhal_backend {
	int    camera_id;
	int    stream_id;
	int    stream_width;
	int    stream_height;
	int    stream_size;
	int    stream_stride;
	int    n_buffers;
	int    running;

	/* Standard SPA logging interface (from create()), may be NULL. */
	struct spa_log *log;

	/* Last-applied 3A settings (persisted so start() can reapply and so a
	 * later set_3a() call has a baseline). */
	struct camhal_3a_settings s3a;

	/* One dqbuf slot we keep cycling: dqbuf -> copy -> qbuf */
	struct camhal_camera_info *cams;    /* (owned/simple) */
	camera_buffer_t **buffers;           /* the qbuf/dqbuf buffer array */

	/* R-A direct / zero-copy mode: when the HAL does not pad lines we let the
	 * caller's mmap'd buffers (which are ALSO the SPA output buffers) serve
	 * directly as the HAL's USERPTR buffers, eliminating the memcpy.  In this
	 * mode buffers[] hold camera_buffer_t wrappers whose ->addr point into the
	 * caller's regions, index i maps 1:1 to buffers[i], and requeue is driven
	 * by camhal_backend_release() (the SPA consumer's reuse_buffer) instead of
	 * the automatic inflight window. */
	int      external;            /* 1 = direct mode (caller-owned buffers) */
	void   **external_addrs;      /* caller-provided region pointers (owned by caller) */
#if ENABLE_DMA_BUF
	int      dma_mode;            /* 1 = R-B DMA-BUF mode (caller-provided dma-buf fds) */
	int     *external_fds;        /* caller-provided DMA-BUF fds (owned by caller) */
#endif
	int     *external_out;        /* per-slot: 1 = currently dequeued, not yet released */
	int      external_head;       /* oldest not-yet-released, for in-order release */

	/*
	 * Multi-buffer in-flight pipeline (P0 fix, mirrors icamerasrc).
	 *
	 * icamerasrc does NOT return a dequeued buffer to the HAL immediately.
	 * The dequeued buffer travels downstream and is only re-queued when the
	 * consumer releases it (release_buffer), so the HAL always has several
	 * buffers in the "dequeued-but-not-yet-requeued" state.  The HAL's
	 * request/3A/PSYS pipeline is driven by every qbuf; with a naive
	 * dqbuf->copy->immediate-qbuf the HAL only ever has ONE buffer in flight,
	 * the request-driving rhythm stalls, and after a couple of seconds the
	 * HAL starts handing back the same stale buffer (content diff=0,
	 * constant timestamp).  We replicate the multi-buffer pipeline here by
	 * keeping up to INFLIGHT_DEPTH dequeued buffers queued up and only
	 * re-queuing the oldest one once the window is full.
	 */
#define INFLIGHT_DEPTH 6
	camera_buffer_t *inflight[INFLIGHT_DEPTH]; /* dequeued-not-requeued buffers */
	int inflight_head;                          /* index of oldest */
	int inflight_count;                         /* how many currently in the window */

	/* DEBUG: previous-source-frame content check (freeze detection) */
	void *prev_src;          /* copy of last buf->addr content compare */
	int   prev_src_size;
	long  dbg_frames;

	pthread_mutex_t lock;
};

long camhal_backend_get_frame_size(int camera_id, uint32_t format,
				   int width, int height)
{
	int bpp = 0;
	return get_frame_size(camera_id, format, width, height,
			      V4L2_FIELD_ANY, &bpp);
}

int camhal_backend_get_supported_formats(int camera_id,
					 struct camhal_resolution *outs,
					 int max_count,
					 struct spa_log *log)
{
	if (!outs || max_count <= 0)
		return -EINVAL;

	camera_info_t cinfo;
	memset(&cinfo, 0, sizeof(cinfo));
	if (get_camera_info(camera_id, cinfo) < 0 || !cinfo.capability)
		return -EIO;

	stream_array_t configs;
	configs.clear();
	cinfo.capability->getSupportedStreamConfig(configs);

	/* Enumerate every distinct (format, width, height) the HAL advertises,
	 * for whatever pixel format it reports (not just NV12). */
	int n = 0;
	for (size_t i = 0; i < configs.size() && n < max_count; i++) {
		/* Skip redundant duplicate entries (same format/size twice). */
		int dup = 0;
		for (int j = 0; j < n; j++) {
			if (outs[j].format  == (uint32_t)configs[i].format &&
			    outs[j].width   == (uint32_t)configs[i].width &&
			    outs[j].height  == (uint32_t)configs[i].height) {
				dup = 1;
				break;
			}
		}
		if (dup)
			continue;
		outs[n].format = configs[i].format;
		outs[n].width  = configs[i].width;
		outs[n].height = configs[i].height;
		n++;
	}

	if (log)
		spa_log_info(log, "camhal: camera %d advertises %d format/size combo(s)",
			     camera_id, n);
	for (int i = 0; i < n && log; i++)
		spa_log_debug(log, "camhal:   [%d] fourcc='%c%c%c%c' %ux%u",
			      i,
			      (char)(outs[i].format & 0xff),
			      (char)((outs[i].format >> 8) & 0xff),
			      (char)((outs[i].format >> 16) & 0xff),
			      (char)((outs[i].format >> 24) & 0xff),
			      outs[i].width, outs[i].height);

	return n;
}

/*
 * Build an icamera::Parameters from a camhal_3a_settings and push it to the
 * HAL.  Zero / keep fields leave the corresponding parameter untouched, so a
 * zeroed struct simply re-applies the auto baseline (AE/AWB AUTO + 30fps),
 * which is what keeps the request/3A/PSYS pipeline from stalling.
 */
static int push_3a(int camera_id, const struct camhal_3a_settings *s,
		   struct spa_log *log)
{
	icamera::Parameters p;
	int ret;

	/* AE mode + manual exposure/gain. */
	if (s->ae_mode != 0) {
		/* treat any non-zero as MANUAL (0 == AUTO default) */
		p.setAeMode(icamera::AE_MODE_MANUAL);
		if (s->apply_exposure && s->exposure_time >= 0)
			p.setExposureTime(s->exposure_time);
		if (s->apply_gain && s->gain > 0)
			p.setSensitivityGain(s->gain);
	} else {
		p.setAeMode(icamera::AE_MODE_AUTO);
	}

	/* AWB mode + manual gains.  With AWB_MODE_MANUAL_GAIN set the HAL uses
	 * setAwbGains() as the white point; otherwise supply gains are ignored
	 * by the HAL, so only send them when the user enabled manual gains. */
	if (s->awb_mode >= 0)
		p.setAwbMode((camera_awb_mode_t)s->awb_mode);
	if (s->apply_awb_gains) {
		icamera::camera_awb_gains_t g;
		g.r_gain = s->awb_r_gain;
		g.g_gain = s->awb_g_gain;
		g.b_gain = s->awb_b_gain;
		p.setAwbGains(g);
	}

	if (s->frame_rate > 0)
		p.setFrameRate(s->frame_rate);
	if (s->run_3a_cadence > 0)
		p.setRun3ACadence(s->run_3a_cadence);

	ret = camera_set_parameters(camera_id, p);
	if (ret != 0) {
		if (log)
			spa_log_warn(log, "camhal: camera_set_parameters ret=%d", ret);
	} else if (log) {
		spa_log_debug(log, "camhal: pushed 3A (ae=%d awb=%d fr=%.0f)",
			      s->ae_mode, s->awb_mode,
			      s->frame_rate > 0 ? s->frame_rate : 30.0f);
	}
	return ret == 0 ? 0 : -EIO;
}

int camhal_backend_set_3a(struct camhal_backend *b,
			  const struct camhal_3a_settings *s)
{
	if (!b || !s)
		return -EINVAL;
	/* Persist the settings so start() can re-apply the same configuration
	 * after a restart without the plugin having to call this again. */
	b->s3a = *s;
	return push_3a(b->camera_id, s, b->log);
}

struct camhal_backend *camhal_backend_create(int camera_id,
					     struct spa_log *log)
{
	struct camhal_backend *b = new (std::nothrow) struct camhal_backend;
	if (!b)
		return NULL;

	memset(b, 0, sizeof(*b));
	b->camera_id = camera_id;
	b->log = log;
	pthread_mutex_init(&b->lock, NULL);

	if (hal_ref() < 0) {
		delete b;
		return NULL;
	}

	if (camera_device_open(camera_id) < 0) {
		hal_unref();
		delete b;
		return NULL;
	}
	CAM_LOG_INFO(b, "camhal: OPEN camera %d (camera_device_open ok)", camera_id);

	/*
	 * Mirror icamerasrc: it calls camera_set_parameters() IMMEDIATELY after
	 * camera_device_open() (gst_camerasrc_start, gstcamerasrc.cpp:2900),
	 * before config_streams.  The HAL's request/3A/PSYS pipeline is wired
	 * up from these baseline parameters; feeding them early (rather than
	 * only at start()) keeps the stats/request rhythm alive.  Use the
	 * persisted 3A settings if the plugin configured any, else the auto
	 * baseline (AE/AWB AUTO, 3A cadence 1, 30 fps) exactly like icamerasrc.
	 */
	{
		struct camhal_3a_settings def;
		memset(&def, 0, sizeof(def));
		def.ae_mode  = 0;             /* AUTO */
		def.awb_mode = -1;            /* keep default (AUTO) */
		/* An all-zero struct would re-apply the auto baseline, but call
		 * push_3a with defaults so the pipeline gets an explicit kick. */
		if (b->s3a.ae_mode != 0 || b->s3a.frame_rate > 0 ||
		    b->s3a.awb_mode >= 0 || b->s3a.apply_awb_gains)
			push_3a(camera_id, &b->s3a, log);
		else
			push_3a(camera_id, &def, log);
	}

	return b;
}

/*
 * Create the USERPTR camera_buffer_t array backing a stream.  In copy mode
 * (external_addrs == NULL) we posix_memalign our own buffers of buf_size bytes;
 * in direct / zero-copy mode (external) we wrap caller-owned mmap'd regions so
 * the HAL writes straight into (what are also) the SPA output buffers.
 *
 * On entry the stream (config_streams) has already been applied and
 * b->stream_stride / b->stream_size are set.  Queues every buffer to the HAL
 * (num_buffers=1 per buffer, the correct single-stream usage).
 *
 * Returns 0 on success, negative errno otherwise (partially built buffers are
 * freed).
 */
static int setup_userptr_buffers(struct camhal_backend *b,
				 const stream_t &stream, int n_buffers,
				 void **external_addrs, size_t addr_size,
				 int buf_size)
{
	size_t bufsz;
	int i;

	/* Determine the per-buffer allocation size.  For external (direct) mode
	 * the caller's regions are used as-is; for copy mode we size to the HAL
	 * frame size page-aligned. */
	if (external_addrs) {
		/* The HAL reports stream.size with conservative allocation slack
		 * even when stride==width (e.g. 461824 for 640x480 whose packed
		 * NV12 is 460800).  What the HAL actually writes is
		 * stride*height*3/2 bytes (stride==width in direct mode = packed
		 * data_size).  Validate the caller's region against that real
		 * written size, NOT the slack-inflated stream.size, so a packed
		 * SPA output buffer (data_size bytes) is accepted. */
		size_t need = (size_t)b->stream_stride * b->stream_height * 3 / 2;
		if (addr_size < need) {
			CAM_LOG_ERR(b, "camhal: external buffer size %zu < need %zu "
				    "(stride=%d h=%d)", addr_size, need,
				    b->stream_stride, b->stream_height);
			return -EINVAL;
		}
		bufsz = addr_size;
		b->external = 1;
		b->external_addrs = external_addrs;
		b->external_out = (int *)calloc(n_buffers, sizeof(int));
		b->external_head = 0;
		if (!b->external_out) {
			b->external = 0;
			return -ENOMEM;
		}
	} else {
		bufsz = (size_t)buf_size;
		/* align up to a page boundary */
		if (bufsz & (getpagesize() - 1))
			bufsz = (bufsz + getpagesize() - 1) &
				~((size_t)getpagesize() - 1);
	}

	b->buffers = (camera_buffer_t **)calloc(n_buffers,
						sizeof(camera_buffer_t *));
	if (!b->buffers) {
		if (b->external_out) {
			free(b->external_out);
			b->external_out = NULL;
		}
		return -ENOMEM;
	}

	for (i = 0; i < n_buffers; i++) {
		camera_buffer_t *buf =
			(camera_buffer_t *)calloc(1, sizeof(camera_buffer_t));
		if (!buf)
			goto err;
		buf->s = stream;
		buf->s.memType = V4L2_MEMORY_USERPTR;
		buf->s.size = (uint32_t)bufsz;
		buf->index = i;

		if (external_addrs) {
			buf->addr = external_addrs[i];
		} else if (posix_memalign(&buf->addr, getpagesize(), bufsz) != 0) {
			CAM_LOG_ERR(b, "camhal: posix_memalign %d failed", i);
			free(buf);
			goto err;
		}
		b->buffers[i] = buf;
		CAM_LOG_DEBUG(b, "camhal: userptr buffer %d addr=%p size=%zu%s",
			      i, buf->addr, bufsz,
			      external_addrs ? " (external/direct)" : "");
	}

	/*
	 * Queue the initial buffers one at a time.
	 *
	 * IMPORTANT: camera_stream_qbuf()'s num_buffers argument is the number
	 * of DIFFERENT streams in the array, NOT the number of buffers.  With a
	 * single stream the HAL maps each queued buffer to that stream, so we
	 * MUST call qbuf with num_buffers=1 once per buffer.  Passing n_buffers
	 * (e.g. 16) told the HAL there were 16 streams, corrupting the mapping
	 * and leaving the stream stuck on the first frame.
	 */
	for (i = 0; i < n_buffers; i++) {
		camera_buffer_t *one = b->buffers[i];
		/* Match icamerasrc: reset sequence/timestamp before every qbuf so
		 * the HAL treats the request as a NEW request (normal 3A) instead
		 * of activating the raw-reprocess branch
		 * (RequestThread::handleRequest checks sequence>=0 && timestamp>0
		 * -> skip 3A, effectSeq goes stale, PSysProcessor::needExecutePipe
		 * returns false -> output buffer never popped -> dqbuf hands back
		 * the same stale buffer = freeze). */
		one->sequence = -1;
		one->timestamp = 0;
		if (camera_stream_qbuf(b->camera_id, &one, 1, NULL) < 0) {
			/* Not an OOM: free local wrappers but never the caller's
			 * external regions. */
			for (int j = 0; j < n_buffers; j++) {
				if (!b->buffers[j])
					continue;
				if (!external_addrs && b->buffers[j]->addr)
					free(b->buffers[j]->addr);
				free(b->buffers[j]);
			}
			free(b->buffers);
			b->buffers = NULL;
			return -EIO;
		}
	}
	return 0;

err:
	for (int j = 0; j < i; j++) {
		if (b->buffers[j]) {
			if (!external_addrs && b->buffers[j]->addr)
				free(b->buffers[j]->addr);
			free(b->buffers[j]);
		}
	}
	free(b->buffers);
	b->buffers = NULL;
	if (b->external_out) {
		free(b->external_out);
		b->external_out = NULL;
	}
	return -ENOMEM;
}

/*
 * Create the V4L2_MEMORY_DMABUF camera_buffer_t array backing a stream (R-B /
 * dma-mode).  The caller owns the DMA-BUF fds (typically exported i915 GEM
 * buffers used as the SPA output buffers); we wrap each in a camera_buffer_t
 * carrying that dmafd so camera_stream_qbuf() imports it via dma_buf_get() and
 * the sensor / PSYS writes the frame straight into the hardware buffer.
 *
 * The HAL must have no line padding (stride == width) for the packed-NV12
 * output layout to match; the caller checks that before calling.
 *
 * Like the USERPTR external path, this sets b->external so the shared
 * dqbuf_index()/release()/stop() index-based tracking applies (those functions
 * only touch ->buffers[index] and ->external_out, never ->addr, so they work
 * unchanged for DMABUF).  The dma fds remain owned by the caller.
 *
 * On entry the stream (config_streams) has been applied and b->stream_stride /
 * b->stream_size are set.  Returns 0 on success, negative errno otherwise.
 */
#if ENABLE_DMA_BUF
static int setup_dmabuf_buffers(struct camhal_backend *b,
				const stream_t &stream, int n_buffers,
				int *fds, int buf_size)
{
	int i;

	if (!fds)
		return -EINVAL;

	/* The HAL reports the (conservative, slack-inflated) stream.size; what it
	 * actually writes with stride==width is stride*height*3/2.  Many GEM /
	 * PRIME buffers are sized to the packed frame (or aligned larger), so
	 * require at least the real written size. */
	{
		size_t need = (size_t)b->stream_stride * b->stream_height * 3 / 2;
		if ((size_t)buf_size < need) {
			CAM_LOG_ERR(b, "camhal: dma buffer size %d < need %zu "
				    "(stride=%d h=%d)", buf_size, need,
				    b->stream_stride, b->stream_height);
			return -EINVAL;
		}
	}

	b->buffers = (camera_buffer_t **)calloc(n_buffers,
						sizeof(camera_buffer_t *));
	if (!b->buffers)
		return -ENOMEM;

	for (i = 0; i < n_buffers; i++) {
		camera_buffer_t *buf =
			(camera_buffer_t *)calloc(1, sizeof(camera_buffer_t));
		if (!buf)
			goto err;
		buf->s = stream;
		buf->s.memType = V4L2_MEMORY_DMABUF;
		buf->s.size = (uint32_t)buf_size;
		buf->dmafd = fds[i];
		buf->flags = BUFFER_FLAG_DMA_EXPORT;
		buf->index = i;
		b->buffers[i] = buf;
		CAM_LOG_DEBUG(b, "camhal: dmabuf buffer %d dmafd=%d size=%d",
			      i, fds[i], buf_size);
	}

	/* Mark direct mode (shares the index-based dqbuf_index/release/stop
	 * tracking with the USERPTR external path). */
	b->dma_mode = 1;
	b->external = 1;
	b->external_fds = fds;
	b->external_out = (int *)calloc(n_buffers, sizeof(int));
	b->external_head = 0;
	if (!b->external_out) {
		b->dma_mode = 0;
		b->external = 0;
		b->external_fds = NULL;
		goto err_free_buffers;
	}

	/* Queue each DMABUF buffer to the HAL, num_buffers=1 per buffer (the
	 * correct single-stream usage; passing n_buffers corrupts the mapping). */
	for (i = 0; i < n_buffers; i++) {
		camera_buffer_t *one = b->buffers[i];
		one->sequence = -1;
		one->timestamp = 0;
		if (camera_stream_qbuf(b->camera_id, &one, 1, NULL) < 0) {
			for (int j = 0; j < n_buffers; j++) {
				if (b->buffers[j])
					free(b->buffers[j]);
			}
			free(b->buffers);
			b->buffers = NULL;
			if (b->external_out) {
				free(b->external_out);
				b->external_out = NULL;
			}
			b->dma_mode = 0;
			b->external = 0;
			b->external_fds = NULL;
			return -EIO;
		}
	}
	return 0;

err:
	for (int j = 0; j < i; j++) {
		if (b->buffers[j])
			free(b->buffers[j]);
	}
err_free_buffers:
	free(b->buffers);
	b->buffers = NULL;
	if (b->external_out) {
		free(b->external_out);
		b->external_out = NULL;
	}
	return -ENOMEM;
}
#endif /* ENABLE_DMA_BUF */

int camhal_backend_configure(struct camhal_backend *b,
			     uint32_t format,
			     int width, int height,
			     int n_buffers,
			     int *out_stride, int *out_size)
{
	if (!b)
		return -EINVAL;

	/* Find the full, HAL-known stream description for (format, width x height).
	 * icamerasrc does exactly this via getSupportedStreamConfig() and copies
	 * the whole stream_t (including stride/size/field/usage) into the array
	 * passed to camera_device_config_streams().  If we hand the HAL a
	 * hand-built stream_t instead, the buffer producer isn't bound correctly:
	 * stride/size come back 0 and camera_device_allocate_memory() fails. */
	camera_info_t cinfo;
	memset(&cinfo, 0, sizeof(cinfo));
	stream_array_t configs;
	stream_t stream;
	memset(&stream, 0, sizeof(stream));
	int found = 0;

	if (get_camera_info(b->camera_id, cinfo) == 0 && cinfo.capability) {
		configs.clear();
		cinfo.capability->getSupportedStreamConfig(configs);
		for (size_t i = 0; i < configs.size(); i++) {
			if ((uint32_t)configs[i].format == format &&
			    configs[i].width == (uint32_t)width &&
			    configs[i].height == (uint32_t)height) {
				stream = configs[i];
				found = 1;
				break;
			}
		}
	}
	if (!found) {
		CAM_LOG_WARN(b, "camhal: no supported fourcc=0x%x %dx%d stream, "
			     "falling back to hand-built stream", format,
			     width, height);
		stream.format    = format;
		stream.width     = width;
		stream.height    = height;
		stream.field     = V4L2_FIELD_ANY;
		stream.memType   = V4L2_MEMORY_MMAP;
		stream.streamType = CAMERA_STREAM_OUTPUT;
		stream.usage     = CAMERA_STREAM_VIDEO_CAPTURE;
		stream.size      = (uint32_t)camhal_backend_get_frame_size(
			b->camera_id, format, width, height);
		if (stream.size <= 0)
			stream.size = (uint32_t)camhal_packed_size(
				format, width, height);
	}
	/* Use USERPTR memory (icamerasrc default); MMAP is only for ISYS
	 * output sensors and fails here.  The plugin memcpys the frame out
	 * of the userptr before recycling it. */
	stream.memType = V4L2_MEMORY_USERPTR;

	stream_config_t config;
	memset(&config, 0, sizeof(config));
	/* The gc5035/ov5675 IPU6ep sensor media control config is registered
	 * with ConfigMode="AUTO".  AUTO lets the HAL select the right McConf
	 * from the stream/resolution internally.  NORMAL (mode 0) does not
	 * match, which made camera_device_config_streams() fail with
	 * "No matching McConf". */
	config.operation_mode = CAMERA_STREAM_CONFIGURATION_MODE_AUTO;
	config.num_streams = 1;
	config.streams = &stream;

	/* icamerasrc calls camera_device_config_sensor_input() BEFORE
	 * camera_device_config_streams(); this tells the HAL to use the
	 * sensor's default ISYS configuration (format = -1 means "default"). */
	stream_t input_config;
	memset(&input_config, 0, sizeof(input_config));
	input_config.format = -1;
	input_config.width  = 0;
	input_config.height = 0;
	camera_device_config_sensor_input(b->camera_id, &input_config);

	if (camera_device_config_streams(b->camera_id, &config) < 0)
		return -EIO;

	CAM_LOG_INFO(b, "camhal: config_streams OK stream.id=%d format=0x%x "
		     "%dx%d stride=%d size=%d field=%d usage=0x%x",
		     stream.id, stream.format, stream.width, stream.height,
		     stream.stride, stream.size, stream.field, stream.usage);

	b->stream_id = stream.id;
	b->stream_width = width;
	b->stream_height = height;
	b->stream_size = stream.size > 0 ? (int)stream.size
					: (int)((size_t)width * height * 3 / 2);
	/* HAL fills stride in stream.stride */
	b->stream_stride = stream.stride > 0 ? stream.stride : width;
	b->n_buffers = n_buffers;

	/*
	 * Allocate and queue n_buffers USERPTR buffers.
	 *
	 * icamerasrc defaults to io-mode=USERPTR (DEFAULT_PROP_IO_MODE), and
	 * that path NEVER calls camera_device_allocate_memory() - the app owns
	 * the memory (posix_memalign) and hands the pointer to the HAL.  The
	 * MMAP path (camera_device_allocate_memory) is only for ISYS-output
	 * sensors; on gc5035/ov5675 (which need a PSYS post-processor to turn
	 * SGRBG10 into NV12) MMAP allocation fails, so we use USERPTR exactly
	 * like icamerasrc does.
	 */
	{
		int ret = setup_userptr_buffers(b, stream, n_buffers,
					       NULL, 0, /* external */
					       b->stream_size);
		if (ret != 0)
			return ret;
	}

	if (out_stride)
		*out_stride = b->stream_stride;
	if (out_size)
		*out_size = b->stream_size;
	return 0;
}

int camhal_backend_start(struct camhal_backend *b)
{
	if (!b)
		return -EINVAL;

	/*
	 * Re-push the 3A parameters (whatever the plugin configured, or the auto
	 * baseline) *before* starting the stream.  icamerasrc always does
	 * camera_set_parameters() in its set_caps / set_property paths, feeding
	 * the HAL's request/3A/stats loop so that PSYS keeps producing fresh
	 * frames.  Without this, the HAL's 3A/statistics pipeline is starved and
	 * camera_stream_dqbuf() begins to hand back the same stale buffer over
	 * and over (content diff=0.0000, constant timestamp) after the first
	 * ~2 s once AE has converged - see the SRC-CONTENT freeze probe.
	 */
	{
		struct camhal_3a_settings def;
		memset(&def, 0, sizeof(def));
		def.awb_mode = -1; /* keep default (AUTO) */
		if (b->s3a.ae_mode != 0 || b->s3a.frame_rate > 0 ||
		    b->s3a.awb_mode >= 0 || b->s3a.apply_awb_gains)
			push_3a(b->camera_id, &b->s3a, b->log);
		else
			push_3a(b->camera_id, &def, b->log);
	}

	if (camera_device_start(b->camera_id) < 0)
		return -EIO;
	b->running = 1;
	return 0;
}

int camhal_backend_dqbuf(struct camhal_backend *b,
			 void *dst, int size,
			 uint64_t *out_ts)
{
	if (!b || !b->running)
		return -EACCES;
	/* Copying capture is only for copy mode; direct mode uses
	 * camhal_backend_dqbuf_index(). */
	if (b->external)
		return -EINVAL;

	camera_buffer_t *buf = NULL;

	/* Block until a frame is ready. */
	if (camera_stream_dqbuf(b->camera_id, b->stream_id, &buf, NULL) < 0)
		return -EIO;

		if (buf && dst && buf->addr) {
			int copy = size;
			if (copy > b->stream_size)
				copy = b->stream_size;
			memcpy(dst, buf->addr, copy);

			/* ---- DEBUG: content freeze check on the SOURCE buffer ---- */
			{
				size_t csize = (size_t)b->stream_size;
				if (csize > 1024ull * 1024)
					csize = 1024ull * 1024; /* 1 MiB cap */
				if (b->prev_src && b->prev_src_size == (int)csize) {
					unsigned long long total = 0;
					unsigned long long cnt = 0;
					const unsigned char *a = (const unsigned char *)buf->addr;
					const unsigned char *c = (const unsigned char *)b->prev_src;
					for (size_t i = 0; i < csize; i += 2) { /* sample */
						long d = (long)a[i] - (long)c[i];
						if (d < 0) d = -d;
						total += (unsigned long long)d;
						cnt++;
					}
					b->dbg_frames++;
					if ((b->dbg_frames % 5) == 1)
						CAM_LOG_DEBUG(b,
							"camhal: SRC-CONTENT diff=%.4f frames=%ld idx=%d seq=%ld frame=%u ts=%lu",
							(double)total / (double)cnt,
							b->dbg_frames, buf->index,
							(long)buf->sequence, buf->frameNumber,
							(unsigned long)buf->timestamp);
				}
				if (!b->prev_src || b->prev_src_size != (int)csize) {
					free(b->prev_src);
					b->prev_src = malloc(csize);
					b->prev_src_size = (int)csize;
				}
				if (b->prev_src && csize)
					memcpy(b->prev_src, buf->addr, csize);
			}
			/* ------------------------------------------------------------ */
			{
				static long nd = 0;
				if ((++nd % 200) == 1)
					CAM_LOG_DEBUG(b,
						"camhal: DQB idx=%d seq=%ld frame=%u ts=%lu",
						buf->index, (long)buf->sequence,
						buf->frameNumber, (unsigned long)buf->timestamp);
			}
		}
	if (out_ts && buf)
		*out_ts = buf->timestamp;

	/*
	 * Multi-buffer in-flight pipeline: do NOT hand the dequeued buffer back
	 * to the HAL right away.  Keep it "in flight" (dequeued-not-requeued) so
	 * the HAL always has several buffers moving through the request/3A/PSYS
	 * pipeline at once, mirroring icamerasrc's bufferpool behaviour.  Only
	 * when the in-flight window is full do we return the OLDEST buffer.
	 */
	if (buf) {
		pthread_mutex_lock(&b->lock);
		/* Window full: return the oldest in-flight buffer first. */
		if (b->inflight_count == INFLIGHT_DEPTH) {
			camera_buffer_t *oldest = b->inflight[b->inflight_head];
			/* Reset sequence/timestamp before requeue (icamerasrc does this
			 * in release_buffer) so HAL treats this as a fresh request and
			 * keeps the 3A/PSYS pipeline running instead of reprocessing a
			 * stale buffer (fixes the ~23-frame freeze). */
			oldest->sequence = -1;
			oldest->timestamp = 0;
			if (camera_stream_qbuf(b->camera_id, &oldest, 1, NULL) < 0)
				CAM_LOG_ERR(b, "camhal: qbuf(inflight) failed");
			else
				CAM_LOG_DEBUG(b,
					"camhal: requeue idx=%d (oldest of %d)",
					oldest->index, INFLIGHT_DEPTH);
			b->inflight_head =
				(b->inflight_head + 1) % INFLIGHT_DEPTH;
			b->inflight_count--;
		}
		/* Park the freshly dequeued buffer at the tail of the window. */
		b->inflight[(b->inflight_head + b->inflight_count) %
			    INFLIGHT_DEPTH] = buf;
		b->inflight_count++;
		pthread_mutex_unlock(&b->lock);
	}

	return 0;
}

int camhal_backend_configure_external(struct camhal_backend *b,
				      uint32_t format,
				      int width, int height,
				      int n_buffers,
				      void **addrs, size_t addr_size,
				      int *out_stride, int *out_size)
{
	int ret;

	if (!b || !addrs || n_buffers <= 0)
		return -EINVAL;

	/* Find the full, HAL-known stream description for (format, width x height),
	 * mirroring camhal_backend_configure.  We need the real table entry so
	 * stride/size come back correct for the zero-copy check. */
	camera_info_t cinfo;
	memset(&cinfo, 0, sizeof(cinfo));
	stream_array_t configs;
	stream_t stream;
	memset(&stream, 0, sizeof(stream));
	int found = 0;

	if (get_camera_info(b->camera_id, cinfo) == 0 && cinfo.capability) {
		configs.clear();
		cinfo.capability->getSupportedStreamConfig(configs);
		for (size_t i = 0; i < configs.size(); i++) {
			if ((uint32_t)configs[i].format == format &&
			    configs[i].width == (uint32_t)width &&
			    configs[i].height == (uint32_t)height) {
				stream = configs[i];
				found = 1;
				break;
			}
		}
	}
	if (!found) {
		CAM_LOG_WARN(b, "camhal: external: no supported fourcc=0x%x %dx%d "
			     "stream", format, width, height);
		return -EINVAL;
	}
	stream.memType = V4L2_MEMORY_USERPTR;

	/*
	 * Direct / zero-copy is ONLY safe when the HAL does not pad each line,
	 * i.e. its bytes-per-line equals the nominal width.  If the HAL would pad
	 * (stride > width) the SPA/consumer layout (packed NV12:
	 * width*height*3/2) does not match the HAL's padded layout and we cannot
	 * hand the consumer's buffers straight to the HAL, so refuse and let the
	 * caller fall back to the copying path.
	 */
	int stride = stream.stride > 0 ? stream.stride : width;
	if (stride != width) {
		CAM_LOG_INFO(b, "camhal: direct mode declined: stride=%d != width=%d "
			     "(would need padding) - falling back to copy path",
			     stride, width);
		return -EINVAL;
	}

	/* Apply the stream configuration exactly like the copying configure. */
	stream_config_t config;
	memset(&config, 0, sizeof(config));
	config.operation_mode = CAMERA_STREAM_CONFIGURATION_MODE_AUTO;
	config.num_streams = 1;
	config.streams = &stream;

	stream_t input_config;
	memset(&input_config, 0, sizeof(input_config));
	input_config.format = -1;
	input_config.width  = 0;
	input_config.height = 0;
	camera_device_config_sensor_input(b->camera_id, &input_config);

	if (camera_device_config_streams(b->camera_id, &config) < 0)
		return -EIO;

	CAM_LOG_INFO(b, "camhal: external config_streams OK id=%d format=0x%x "
		     "%dx%d stride=%d size=%d", stream.id, stream.format,
		     stream.width, stream.height, stream.stride, stream.size);

	b->stream_id = stream.id;
	b->stream_width = width;
	b->stream_height = height;
	b->stream_size = stream.size > 0 ? (int)stream.size
					: (int)camhal_packed_size(
						format, width, height);
	b->stream_stride = stride;
	b->n_buffers = n_buffers;

	CAM_LOG_INFO(b, "camhal: direct mode: HAL stride==width (%d), "
		     "zero-copy via caller USERPTR buffers", stride);

	/* Register the caller's mmap'd buffers as the USERPTR array. */
	ret = setup_userptr_buffers(b, stream, n_buffers,
				    addrs, addr_size, 0);
	if (ret != 0)
		return ret;

	if (out_stride)
		*out_stride = b->stream_stride;
	if (out_size)
		*out_size = b->stream_size;
	return 0;
}

#if ENABLE_DMA_BUF
int camhal_backend_configure_dmabuf(struct camhal_backend *b,
				    uint32_t format,
				    int width, int height,
				    int n_buffers,
				    int *fds,
				    int *out_stride, int *out_size)
{
	int ret;

	if (!b || !fds || n_buffers <= 0)
		return -EINVAL;

	/* Find the full, HAL-known stream description for (format, width x height),
	 * mirroring camhal_backend_configure / _external. */
	camera_info_t cinfo;
	memset(&cinfo, 0, sizeof(cinfo));
	stream_array_t configs;
	stream_t stream;
	memset(&stream, 0, sizeof(stream));
	int found = 0;

	if (get_camera_info(b->camera_id, cinfo) == 0 && cinfo.capability) {
		configs.clear();
		cinfo.capability->getSupportedStreamConfig(configs);
		for (size_t i = 0; i < configs.size(); i++) {
			if ((uint32_t)configs[i].format == format &&
			    configs[i].width == (uint32_t)width &&
			    configs[i].height == (uint32_t)height) {
				stream = configs[i];
				found = 1;
				break;
			}
		}
	}
	if (!found) {
		CAM_LOG_WARN(b, "camhal: dma-mode: no supported fourcc=0x%x %dx%d "
			     "stream", format, width, height);
		return -EINVAL;
	}
	stream.memType = V4L2_MEMORY_DMABUF;

	/* Like R-A external, DMA-BUF passthrough is ONLY safe when the HAL does
	 * not pad each line (stride == width), so the packed NV12 layout matches
	 * what the (SPA) buffer holds.  Refuse otherwise and let the caller
	 * fall back to the copying path. */
	int stride = stream.stride > 0 ? stream.stride : width;
	if (stride != width) {
		CAM_LOG_INFO(b, "camhal: dma-mode declined: stride=%d != width=%d "
			     "(would need padding) - falling back to copy path",
			     stride, width);
		return -EINVAL;
	}

	stream_config_t config;
	memset(&config, 0, sizeof(config));
	config.operation_mode = CAMERA_STREAM_CONFIGURATION_MODE_AUTO;
	config.num_streams = 1;
	config.streams = &stream;

	stream_t input_config;
	memset(&input_config, 0, sizeof(input_config));
	input_config.format = -1;
	input_config.width  = 0;
	input_config.height = 0;
	camera_device_config_sensor_input(b->camera_id, &input_config);

	if (camera_device_config_streams(b->camera_id, &config) < 0)
		return -EIO;

	CAM_LOG_INFO(b, "camhal: dma-mode config_streams OK id=%d format=0x%x "
		     "%dx%d stride=%d size=%d", stream.id, stream.format,
		     stream.width, stream.height, stream.stride, stream.size);

	b->stream_id = stream.id;
	b->stream_width = width;
	b->stream_height = height;
	b->stream_size = stream.size > 0 ? (int)stream.size
					: (int)camhal_packed_size(
						format, width, height);
	b->stream_stride = stride;
	b->n_buffers = n_buffers;

	/* DMA-BUF buffers have no CPU address; use a page-aligned frame size as
	 * the per-buffer size the HAL validation expects. */
	int bufsz = (int)b->stream_size;
	if (bufsz & (getpagesize() - 1))
		bufsz = (bufsz + getpagesize() - 1) & ~(getpagesize() - 1);

	CAM_LOG_INFO(b, "camhal: dma-mode active: HAL stride==width (%d), "
		     "zero-copy via caller DMA-BUF import", stride);

	/* Register the caller's DMA-BUF fds as the DMABUF array. */
	ret = setup_dmabuf_buffers(b, stream, n_buffers, fds, bufsz);
	if (ret != 0)
		return ret;

	if (out_stride)
		*out_stride = b->stream_stride;
	if (out_size)
		*out_size = b->stream_size;
	return 0;
}
#endif /* ENABLE_DMA_BUF */

int camhal_backend_dqbuf_index(struct camhal_backend *b,
			       int *out_index, uint64_t *out_ts)
{
	camera_buffer_t *buf = NULL;

	if (!b || !b->running || !b->external || !out_index)
		return -EINVAL;

	/* Block until a frame lands in one of the direct-mode USERPTR buffers. */
	if (camera_stream_dqbuf(b->camera_id, b->stream_id, &buf, NULL) < 0)
		return -EIO;

	if (buf->index < 0 || buf->index >= b->n_buffers) {
		CAM_LOG_ERR(b, "camhal: dqbuf_index returned bad index %d (n=%d)",
			    buf->index, b->n_buffers);
		/* Try to hand it back so we don't leak it. */
		buf->sequence = -1;
		buf->timestamp = 0;
		camera_stream_qbuf(b->camera_id, &buf, 1, NULL);
		return -EIO;
	}

	{
		static long nd = 0;
		if ((++nd % 200) == 1)
			CAM_LOG_DEBUG(b, "camhal: DQB-idx=%d seq=%ld frame=%u ts=%lu",
				      buf->index, (long)buf->sequence,
				      buf->frameNumber,
				      (unsigned long)buf->timestamp);
	}

	pthread_mutex_lock(&b->lock);
	b->external_out[buf->index] = 1;
	pthread_mutex_unlock(&b->lock);

	if (out_ts)
		*out_ts = buf->timestamp;
	*out_index = buf->index;
	return 0;
}

int camhal_backend_release(struct camhal_backend *b, int index)
{
	camera_buffer_t *buf;

	if (!b || !b->external || !b->buffers)
		return -EINVAL;
	if (index < 0 || index >= b->n_buffers)
		return -EINVAL;

	pthread_mutex_lock(&b->lock);
	if (!b->external_out[index]) {
		pthread_mutex_unlock(&b->lock);
		/* Double-release: ignore (idempotent). */
		return 0;
	}
	b->external_out[index] = 0;
	pthread_mutex_unlock(&b->lock);

	buf = b->buffers[index];
	if (!buf)
		return -EINVAL;

	/* Match icamerasrc release_buffer: reset sequence/timestamp so the HAL
	 * treats this as a fresh request and keeps 3A/PSYS producing. */
	buf->sequence = -1;
	buf->timestamp = 0;
	if (camera_stream_qbuf(b->camera_id, &buf, 1, NULL) < 0) {
		CAM_LOG_ERR(b, "camhal: release qbuf idx=%d failed", index);
		return -EIO;
	}
	CAM_LOG_DEBUG(b, "camhal: released idx=%d", index);
	return 0;
}

int camhal_backend_stop(struct camhal_backend *b)
{
	if (!b)
		return -EINVAL;
	if (b->running) {
		if (b->external) {
			/* Direct mode: hand any dequeued-not-yet-released buffers
			 * back to the HAL so it has all its buffers before stop. */
			pthread_mutex_lock(&b->lock);
			for (int i = 0; i < b->n_buffers; i++) {
				if (b->external_out && b->external_out[i] &&
				    b->buffers && b->buffers[i]) {
					camera_buffer_t *one = b->buffers[i];
					one->sequence = -1;
					one->timestamp = 0;
					camera_stream_qbuf(b->camera_id, &one, 1, NULL);
					b->external_out[i] = 0;
				}
			}
			pthread_mutex_unlock(&b->lock);
		} else {
			/* Return any in-flight (dequeued-not-requeued) buffers to
			 * the HAL before stopping, so the HAL has all its buffers. */
			pthread_mutex_lock(&b->lock);
			while (b->inflight_count > 0) {
				camera_buffer_t *oldest = b->inflight[b->inflight_head];
				if (oldest) {
					oldest->sequence = -1;
					oldest->timestamp = 0;
					camera_stream_qbuf(b->camera_id, &oldest, 1, NULL);
				}
				b->inflight_head =
					(b->inflight_head + 1) % INFLIGHT_DEPTH;
				b->inflight_count--;
			}
			pthread_mutex_unlock(&b->lock);
		}
		camera_device_stop(b->camera_id);
		b->running = 0;
	}
	return 0;
}

void camhal_backend_destroy(struct camhal_backend *b)
{
	if (!b)
		return;
	camhal_backend_stop(b);
	if (b->buffers) {
		for (int i = 0; i < b->n_buffers; i++) {
			if (b->buffers[i]) {
				/* In external/direct mode the ->addr regions are
				 * owned by the caller (the SPA output buffers) - do
				 * NOT free them.  Only our own copy-mode buffers. */
				if (!b->external && b->buffers[i]->addr)
					free(b->buffers[i]->addr);
				free(b->buffers[i]);
			}
		}
		free(b->buffers);
		b->buffers = NULL;
	}
	free(b->external_out);
	b->external_out = NULL;
	CAM_LOG_INFO(b, "camhal: CLOSE camera %d (camera_device_close)", b->camera_id);
	camera_device_close(b->camera_id);
	free(b->prev_src);
	b->prev_src = NULL;
	hal_unref();
	CAM_LOG_INFO(b, "camhal: DESTROY backend camera %d done (hal_unref)", b->camera_id);
	pthread_mutex_destroy(&b->lock);
	delete b;
}
