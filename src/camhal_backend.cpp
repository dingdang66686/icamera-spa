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

#include <libcamhal/api/ICamera.h>
#include <libcamhal/api/Parameters.h>
#include <libcamhal/linux/videodev2.h>

using namespace icamera;

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

	/* Last-applied 3A settings (persisted so start() can reapply and so a
	 * later set_3a() call has a baseline). */
	struct camhal_3a_settings s3a;

	/* One dqbuf slot we keep cycling: dqbuf -> copy -> qbuf */
	struct camhal_camera_info *cams;    /* (owned/simple) */
	camera_buffer_t **buffers;           /* the qbuf/dqbuf buffer array */

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

long camhal_backend_get_frame_size(int camera_id, int width, int height)
{
	int bpp = 0;
	return get_frame_size(camera_id, V4L2_PIX_FMT_NV12, width, height,
			      V4L2_FIELD_ANY, &bpp);
}

int camhal_backend_get_supported_formats(int camera_id,
					 struct camhal_resolution *outs,
					 int max_count)
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

	int n = 0;
	for (size_t i = 0; i < configs.size() && n < max_count; i++) {
		if (configs[i].format != V4L2_PIX_FMT_NV12)
			continue;
		/* Skip redundant duplicate resolutions (same size twice). */
		int dup = 0;
		for (int j = 0; j < n; j++) {
			if (outs[j].width == (uint32_t)configs[i].width &&
			    outs[j].height == (uint32_t)configs[i].height) {
				dup = 1;
				break;
			}
		}
		if (dup)
			continue;
		outs[n].width  = configs[i].width;
		outs[n].height = configs[i].height;
		n++;
	}

	fprintf(stderr, "camhal: camera %d supports %d NV12 resolution(s)\n",
		camera_id, n);
	for (int i = 0; i < n; i++)
		fprintf(stderr, "camhal:   [%d] %ux%u\n",
			i, outs[i].width, outs[i].height);

	return n;
}

/*
 * Build an icamera::Parameters from a camhal_3a_settings and push it to the
 * HAL.  Zero / keep fields leave the corresponding parameter untouched, so a
 * zeroed struct simply re-applies the auto baseline (AE/AWB AUTO + 30fps),
 * which is what keeps the request/3A/PSYS pipeline from stalling.
 */
static int push_3a(int camera_id, const struct camhal_3a_settings *s)
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
	if (ret != 0)
		fprintf(stderr, "camhal: camera_set_parameters ret=%d\n", ret);
	else
		fprintf(stderr, "camhal: pushed 3A (ae=%d awb=%d fr=%.0f)"
			"\n", s->ae_mode, s->awb_mode,
			s->frame_rate > 0 ? s->frame_rate : 30.0f);
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
	return push_3a(b->camera_id, s);
}

struct camhal_backend *camhal_backend_create(int camera_id)
{
	struct camhal_backend *b = new (std::nothrow) struct camhal_backend;
	if (!b)
		return NULL;

	memset(b, 0, sizeof(*b));
	b->camera_id = camera_id;
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
			push_3a(camera_id, &b->s3a);
		else
			push_3a(camera_id, &def);
	}

	return b;
}

int camhal_backend_configure(struct camhal_backend *b,
			     int width, int height,
			     int n_buffers,
			     int *out_stride, int *out_size)
{
	if (!b)
		return -EINVAL;

	/* Find the full, HAL-known stream description for (NV12, width x height).
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
			if (configs[i].format == V4L2_PIX_FMT_NV12 &&
			    configs[i].width == (uint32_t)width &&
			    configs[i].height == (uint32_t)height) {
				stream = configs[i];
				found = 1;
				break;
			}
		}
	}
	if (!found) {
		fprintf(stderr, "camhal: no supported NV12 %dx%d stream, "
			"falling back to hand-built stream\n", width, height);
		stream.format    = V4L2_PIX_FMT_NV12;
		stream.width     = width;
		stream.height    = height;
		stream.field     = V4L2_FIELD_ANY;
		stream.memType   = V4L2_MEMORY_MMAP;
		stream.streamType = CAMERA_STREAM_OUTPUT;
		stream.usage     = CAMERA_STREAM_VIDEO_CAPTURE;
		stream.size      = (uint32_t)camhal_backend_get_frame_size(
			b->camera_id, width, height);
		if (stream.size <= 0)
			stream.size = (uint32_t)((size_t)width * height * 3 / 2);
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

	fprintf(stderr, "camhal: config_streams OK stream.id=%d format=0x%x "
		"%dx%d stride=%d size=%d field=%d usage=0x%x\n",
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
	b->buffers = (camera_buffer_t **)calloc(n_buffers, sizeof(camera_buffer_t *));
	if (!b->buffers)
		return -ENOMEM;

	size_t bufsz = (size_t)b->stream_size;
	/* align up to a page boundary */
	if (bufsz & (getpagesize() - 1))
		bufsz = (bufsz + getpagesize() - 1) & ~((size_t)getpagesize() - 1);

	for (int i = 0; i < n_buffers; i++) {
		camera_buffer_t *buf = (camera_buffer_t *)calloc(1, sizeof(camera_buffer_t));
		if (!buf) {
			for (int j = 0; j < i; j++) {
				if (b->buffers[j]) {
					if (b->buffers[j]->addr)
						free(b->buffers[j]->addr);
					free(b->buffers[j]);
				}
			}
			free(b->buffers);
			b->buffers = NULL;
			return -ENOMEM;
		}
		buf->s = stream;
		buf->s.memType = V4L2_MEMORY_USERPTR;
		buf->s.size = (uint32_t)bufsz;
		buf->index = i;

		if (posix_memalign(&buf->addr, getpagesize(), bufsz) != 0) {
			fprintf(stderr, "camhal: posix_memalign %d failed\n", i);
			free(buf);
			for (int j = 0; j < i; j++) {
				if (b->buffers[j]) {
					if (b->buffers[j]->addr)
						free(b->buffers[j]->addr);
					free(b->buffers[j]);
				}
			}
			free(b->buffers);
			b->buffers = NULL;
			return -ENOMEM;
		}
		b->buffers[i] = buf;
		fprintf(stderr, "camhal: userptr buffer %d addr=%p size=%zu\n",
			i, buf->addr, bufsz);
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
	for (int i = 0; i < n_buffers; i++) {
		camera_buffer_t *one = b->buffers[i];
		/* Match icamerasrc: reset sequence/timestamp before every qbuf so
		 * the HAL treats the request as a NEW request (normal 3A) instead
		 * of activating the raw-reprocess branch (RequestThread::handleRequest
		 * checks sequence>=0 && timestamp>0 -> skip 3A, effectSeq goes stale,
		 * PSysProcessor::needExecutePipe returns false -> output buffer never
		 * popped -> dqbuf hands back the same stale buffer = freeze). */
		one->sequence = -1;
		one->timestamp = 0;
		if (camera_stream_qbuf(b->camera_id, &one, 1, NULL) < 0)
			return -EIO;
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
			push_3a(b->camera_id, &b->s3a);
		else
			push_3a(b->camera_id, &def);
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
						fprintf(stderr,
							"camhal: SRC-CONTENT diff=%.4f frames=%ld idx=%d seq=%ld frame=%u ts=%lu\n",
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
					fprintf(stderr, "camhal: DQB idx=%d seq=%ld frame=%u ts=%lu\n",
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
				fprintf(stderr, "camhal: qbuf(inflight) failed\n");
			else
				fprintf(stderr,
					"camhal: requeue idx=%d (oldest of %d)\n",
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

int camhal_backend_stop(struct camhal_backend *b)
{
	if (!b)
		return -EINVAL;
	if (b->running) {
		/* Return any in-flight (dequeued-not-requeued) buffers to the HAL
		 * before stopping, so the HAL has all its buffers back. */
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
				if (b->buffers[i]->addr)
					free(b->buffers[i]->addr);
				free(b->buffers[i]);
			}
		}
		free(b->buffers);
		b->buffers = NULL;
	}
	camera_device_close(b->camera_id);
	free(b->prev_src);
	b->prev_src = NULL;
	hal_unref();
	pthread_mutex_destroy(&b->lock);
	delete b;
}
