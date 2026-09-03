/* icamera-format.c - V4L2 fourcc / SPA video format helpers (pure).
 *
 * Pure helper functions shared by the icamera SPA node.  None of these
 * touch node/port/backend state, so they live in their own translation
 * unit and can be unit-tested independently.
 */
#include "icamera-format.h"

#include <math.h>

#include <spa/param/video/format.h>

uint32_t v4l2_fourcc_to_spa(uint32_t fourcc)
{
	switch (fourcc) {
	case V4L2_FOURCC('N', 'V', '1', '2'): return SPA_VIDEO_FORMAT_NV12;
	case V4L2_FOURCC('N', 'V', '2', '1'): return SPA_VIDEO_FORMAT_NV21;
	case V4L2_FOURCC('Y', 'U', 'Y', 'V'): return SPA_VIDEO_FORMAT_YUY2;
	case V4L2_FOURCC('U', 'Y', 'V', 'Y'): return SPA_VIDEO_FORMAT_UYVY;
	case V4L2_FOURCC('Y', 'U', '1', '2'): return SPA_VIDEO_FORMAT_I420;
	case V4L2_FOURCC('Y', 'V', '1', '2'): return SPA_VIDEO_FORMAT_YV12;
	case V4L2_FOURCC('G', 'R', 'E', 'Y'): return SPA_VIDEO_FORMAT_GRAY8;
	case V4L2_FOURCC('R', 'G', 'B', '3'): return SPA_VIDEO_FORMAT_RGB;
	case V4L2_FOURCC('B', 'G', 'R', '3'): return SPA_VIDEO_FORMAT_BGR;
	case V4L2_FOURCC('R', 'G', 'B', 'P'): return SPA_VIDEO_FORMAT_RGB16;
	default:                              return SPA_VIDEO_FORMAT_UNKNOWN;
	}
}

uint32_t spa_format_to_v4l2_fourcc(uint32_t fmt)
{
	switch (fmt) {
	case SPA_VIDEO_FORMAT_NV12:  return V4L2_FOURCC('N', 'V', '1', '2');
	case SPA_VIDEO_FORMAT_NV21:  return V4L2_FOURCC('N', 'V', '2', '1');
	case SPA_VIDEO_FORMAT_YUY2:  return V4L2_FOURCC('Y', 'U', 'Y', 'V');
	case SPA_VIDEO_FORMAT_UYVY:  return V4L2_FOURCC('U', 'Y', 'V', 'Y');
	case SPA_VIDEO_FORMAT_I420:  return V4L2_FOURCC('Y', 'U', '1', '2');
	case SPA_VIDEO_FORMAT_YV12:  return V4L2_FOURCC('Y', 'V', '1', '2');
	case SPA_VIDEO_FORMAT_GRAY8: return V4L2_FOURCC('G', 'R', 'E', 'Y');
	case SPA_VIDEO_FORMAT_RGB:   return V4L2_FOURCC('R', 'G', 'B', '3');
	case SPA_VIDEO_FORMAT_BGR:   return V4L2_FOURCC('B', 'G', 'R', '3');
	case SPA_VIDEO_FORMAT_RGB16: return V4L2_FOURCC('R', 'G', 'B', 'P');
	default:                     return 0;
	}
}

size_t v4l2_format_size(uint32_t fourcc, uint32_t w, uint32_t h)
{
	size_t px = (size_t)w * h;
	switch (fourcc) {
	case V4L2_FOURCC('N', 'V', '1', '2'):
	case V4L2_FOURCC('N', 'V', '2', '1'):
	case V4L2_FOURCC('Y', 'U', '1', '2'):
	case V4L2_FOURCC('Y', 'V', '1', '2'):
		return px * 3 / 2;
	case V4L2_FOURCC('Y', 'U', 'Y', 'V'):
	case V4L2_FOURCC('U', 'Y', 'V', 'Y'):
	case V4L2_FOURCC('R', 'G', 'B', 'P'):
		return px * 2;
	case V4L2_FOURCC('R', 'G', 'B', '3'):
	case V4L2_FOURCC('B', 'G', 'R', '3'):
		return px * 3;
	case V4L2_FOURCC('G', 'R', 'E', 'Y'):
		return px;
	default:
		return px * 3;
	}
}

void icamera_fps_to_fraction(float fps, struct spa_fraction *frac)
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
