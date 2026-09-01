/*
 * camhal_backend.h - C interface to the Intel libcamhal backend
 *
 * This is a thin C bridge: the PipeWire SPA plugin (written in C) calls
 * these functions, which are implemented in C++ (camhal_backend.cpp) and
 * talk directly to the Intel IPU6 Camera HAL library (icamera:: namespace).
 *
 * The backend runs a blocking qbuf/dqbuf capture loop and copies each
 * dequeued NV12 frame into a caller-provided destination buffer.  This
 * deliberately mirrors the old GStreamer appsink producer: the SPA plugin
 * treats the backend as "give me the next frame".
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CAMHAL_BACKEND_H
#define CAMHAL_BACKEND_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration of the SPA logging interface (<spa/support/log.h>).
 * The backend logs through this standard channel so its messages land in
 * the same PipeWire pw_log stream as the SPA node, with level filtering
 * instead of raw fprintf(stderr).  May be NULL (logging disabled). */
struct spa_log;

/* Opaque handle to one configured+startable camera stream. */
struct camhal_backend;

/* A discovered camera. */
struct camhal_camera_info {
	int      camera_id;   /* CamHAL device index (e.g. 4 = gc5035-uf, 5 = ov5675-uf) */
	int      facing;      /* 0 = rear, 1 = front */
	char     name[64];    /* sensor name, e.g. "gc5035-uf" */
	char     description[128];
};

/*
 * Discover the real built-in cameras (excludes USB / UVC raw sensors).
 * Returns an allocated array (caller frees with camhal_free_cameras()).
 * \param out_count  receives the number of cameras
 * \return  malloc'd struct camhal_camera_info array, or NULL on error.
 */
struct camhal_camera_info *camhal_discover_cameras(int *out_count);

/* Free the array returned by camhal_discover_cameras(). */
void camhal_free_cameras(struct camhal_camera_info *cams, int count);

/*
 * Create a backend for the given CamHAL camera_id.
 * This performs camera_hal_init() (ref-counted) + camera_device_open().
 * \param log  SPA logging interface to emit diagnostics through
 *             (optional, may be NULL to silence non-fatal messages).
 * Returns NULL on failure.  Destroys with camhal_backend_destroy().
 */
struct camhal_backend *camhal_backend_create(int camera_id,
					     struct spa_log *log);

/*
 * Query the frame size the HAL wants for (format NV12, width x height).
 * Optional; the plugin can also just assume width*height*3/2.
 */
long camhal_backend_get_frame_size(int camera_id, int width, int height);

/* One supported NV12 video resolution advertised by the HAL. */
struct camhal_resolution {
	uint32_t width;
	uint32_t height;
};

/*
 * Enumerate the NV12 resolutions the HAL actually supports for the given
 * camera.  Fills at most \a max_count entries into \a outs and returns the
 * number written (<= max_count).  Returns negative errno on failure.
 * \param log  optional SPA logging interface for diagnostics (may be NULL).
 *
 * This queries getSupportedStreamConfig(), so it does NOT open the camera
 * (no camera_device_open()/config_streams() needed) and therefore does not
 * hold the device - safe to call during node init.
 */
int camhal_backend_get_supported_formats(int camera_id,
					 struct camhal_resolution *outs,
					 int max_count,
					 struct spa_log *log);

/*
 * Tunable 3A parameters.  Zero / negative "keep" values mean "don't change
 * this parameter, keep the backend default"; the apply_* flags select which
 * fields are actually written to the HAL.  Set the whole struct to zero and
 * call camhal_backend_set_3a() to (re)apply the default auto baseline.
 */
struct camhal_3a_settings {
	int      ae_mode;         /* 0 = AE_MODE_AUTO, 1 = AE_MODE_MANUAL */
	int      apply_exposure;  /* 1 = write exposure_time below */
	int64_t  exposure_time;   /* ns, only meaningful when ae_mode=MANUAL */
	int      apply_gain;      /* 1 = write gain below */
	float    gain;            /* sensitivity gain (ISO), ae manual */
	int      awb_mode;        /* camera_awb_mode_t, -1 = keep default */
	int      apply_awb_gains; /* 1 = write manual AWB gains below */
	int      awb_r_gain;      /* manual AWB gain R  (only if apply_awb_gains) */
	int      awb_g_gain;      /* manual AWB gain G  (only if apply_awb_gains) */
	int      awb_b_gain;      /* manual AWB gain B  (only if apply_awb_gains) */
	float    frame_rate;      /* fps, <=0 = keep default */
	int      run_3a_cadence;  /* 3A cadence, <=0 = keep default */
};

/*
 * Apply 3A settings to the given (already-open) backend's camera via
 * camera_set_parameters().  Safe to call before or during streaming (the
 * HAL picks up parameter changes on the next request).  Returns 0 on
 * success, negative errno on failure.
 */
int camhal_backend_set_3a(struct camhal_backend *b,
			  const struct camhal_3a_settings *s);

/*
 * Configure the stream.  Must be called once, while stopped.
 * width/height in pixels, n_buffers = number of streaming buffers.
 * \param out_stride receives the bytes-per-line the HAL will use.
 */
int camhal_backend_configure(struct camhal_backend *b,
			     int width, int height,
			     int n_buffers,
			     int *out_stride, int *out_size);

/*
 * Start the sensor (camera_device_start).  Call after configure.
 */
int camhal_backend_start(struct camhal_backend *b);

/*
 * Blocking capture of the next frame into dst (copies size bytes).
 * This internally queues the previously dequeued buffer, dequeues a new
 * one, copies it to dst and re-queues it.
 *   dst  - caller-provided buffer of at least size bytes
 *   size - expected frame size (NV12 = width*height*3/2)
 *   out_ts - receives the buffer timestamp (ns) if non-NULL
 * Returns 0 on success, negative errno on failure.
 */
int camhal_backend_dqbuf(struct camhal_backend *b,
			 void *dst, int size,
			 uint64_t *out_ts);

/*
 * Stop (camera_device_stop) and release resources (close + deinit).
 */
int camhal_backend_stop(struct camhal_backend *b);
void camhal_backend_destroy(struct camhal_backend *b);

#ifdef __cplusplus
}
#endif

#endif /* CAMHAL_BACKEND_H */
