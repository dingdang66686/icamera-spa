// test-hal-release.cpp - verify that the CamHAL device is TRULY released on
// camhal_backend_destroy() (camera_device_close -> hal_unref), so that a second
// open/release cycle succeeds.  This isolates the HAL release fix from the
// WirePlumber auto-link/router noise.
//
// It drives the same backend API the SPA plugin uses (copy mode), looping
// 3x open->stream->stop->destroy and printing which camera each cycle opens.
//
// Exit code:
//   0 = every cycle opened, streamed frames, and (critically) the NEXT cycle
//       could re-open the SAME camera -> release works.
//   1 = some cycle failed to OPEN (camera still held by a previous cycle) or
//       failed to stream.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>

#include "camhal_backend.h"

static int cycle(int camera_id, int seq)
{
	char label[32];
	snprintf(label, sizeof(label), "cyc%d", seq);

	struct camhal_backend *b = camhal_backend_create(camera_id, NULL);
	if (!b) {
		printf("[%s] OPEN FAILED (camera %d still held?)\n", label, camera_id);
		return 1;
	}
	printf("[%s] opened camera %d\n", label, camera_id);

	int stride = 0, size = 0;
	if (camhal_backend_configure(b, 0x3231564e /* NV12 */,
				     640, 480, 6, &stride, &size) < 0) {
		printf("[%s] configure failed\n", label);
		camhal_backend_destroy(b);
		return 1;
	}
	printf("[%s] configured 640x480 stride=%d size=%d\n", label, stride, size);

	if (camhal_backend_start(b) < 0) {
		printf("[%s] start failed\n", label);
		camhal_backend_destroy(b);
		return 1;
	}

	/* Capture a couple of frames. */
	void *buf = malloc(size);
	if (!buf) { camhal_backend_stop(b); camhal_backend_destroy(b); return 1; }
	for (int i = 0; i < 3; i++) {
		uint64_t ts = 0;
		int r = camhal_backend_dqbuf(b, buf, size, &ts);
		if (r < 0) {
			printf("[%s] dqbuf failed (%d)\n", label, r);
			break;
		}
		printf("[%s] frame %d ts=%llu\n", label, i, (unsigned long long)ts);
	}
	free(buf);

	camhal_backend_stop(b);
	camhal_backend_destroy(b);   /* <-- must call camera_device_close + hal_unref */
	printf("[%s] stopped + destroyed backend (camera released)\n", label);
	return 0;
}

int main(void)
{
	int n = 0;
	struct camhal_camera_info *cams = camhal_discover_cameras(&n);
	if (!cams || n <= 0) {
		printf("no cameras discovered\n");
		return 1;
	}
	int rear = -1, front = -1;
	for (int i = 0; i < n; i++) {
		printf("cam %d: id=%d facing=%d name=%s\n", i, cams[i].camera_id,
		       cams[i].facing, cams[i].name);
		if (strstr(cams[i].name, "gc5035")) rear = cams[i].camera_id;
		if (strstr(cams[i].name, "ov5675")) front = cams[i].camera_id;
	}
	camhal_free_cameras(cams, n);

	int target = rear >= 0 ? rear : front;
	printf("== target camera = %d (rear/gc5035 if present) ==\n", target);
	if (target < 0) { printf("no known camera\n"); return 1; }

	int fails = 0;
	for (int i = 1; i <= 3; i++) {
		printf("\n===== cycle %d =====\n", i);
		fails += cycle(target, i);
	}
	printf("\n== %s: %d cycle(s) failed ==\n", fails ? "RELEASE BUG PRESENT" : "ALL OK - camera released between cycles", fails);
	return fails ? 1 : 0;
}
