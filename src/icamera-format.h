/* icamera-format.h - V4L2 fourcc / SPA video format helpers (pure) */
#ifndef ICAMERA_FORMAT_H
#define ICAMERA_FORMAT_H

#include <stdint.h>
#include <stddef.h>

#include <spa/utils/defs.h>

#ifdef __cplusplus
extern "C" {
#endif

#define V4L2_FOURCC(a, b, c, d) \
	((uint32_t)(a) | ((uint32_t)(b) << 8) | \
	 ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

/* Map a V4L2 pixel fourcc to the equivalent SPA_VIDEO_FORMAT_* id, or
 * SPA_VIDEO_FORMAT_UNKNOWN when we do not represent it in SPA. */
uint32_t v4l2_fourcc_to_spa(uint32_t fourcc);

/* Map a negotiated SPA_VIDEO_FORMAT_* id back to its V4L2 fourcc, or 0
 * when unknown.  Inverse of v4l2_fourcc_to_spa() for the formats above. */
uint32_t spa_format_to_v4l2_fourcc(uint32_t fmt);

/* Packed byte size of a V4L2 fourcc frame (no line padding).  Handles the
 * formats we advertise; unknown formats fall back to 3 bytes/pixel. */
size_t v4l2_format_size(uint32_t fourcc, uint32_t w, uint32_t h);

/* Convert a target frame rate in fps (float) to a struct spa_fraction,
 * snapping to the nearest common integer/fractional CRT frame rate.
 * fps <= 0 means "use the HAL default" and falls back to 30/1.
 *   e.g. 30.0 -> {30,1}  60.0 -> {60,1}  25.0 -> {25,1}
 *        29.97 -> {30000,1001}  59.94 -> {60000,1001}  23.976 -> {24000,1001}
 */
void icamera_fps_to_fraction(float fps, struct spa_fraction *frac);

#ifdef __cplusplus
}
#endif

#endif /* ICAMERA_FORMAT_H */
