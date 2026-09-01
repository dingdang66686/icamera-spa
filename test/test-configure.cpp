// test-configure.cpp - diagnose camhal_backend create + configure on the
// real IPU6 hardware.
//
// Build:
//   g++ -O2 -Wall -Isrc -o build/test-configure test/test-configure.cpp \
//       ./build/libspa-icamera.so
#include <stdio.h>
#include "camhal_backend.h"

int main(void)
{
	int count = 0;
	struct camhal_camera_info *cams = camhal_discover_cameras(&count);
	if (!cams) { printf("discover failed\n"); return 1; }
	for (int i = 0; i < count; i++)
		printf("cam %d: id=%d facing=%d %s\n", i, cams[i].camera_id,
		       cams[i].facing, cams[i].name);

	for (int i = 0; i < count; i++) {
		int id = cams[i].camera_id;
		printf("\n=== create(%d) ===\n", id);
		struct camhal_backend *b = camhal_backend_create(id);
		if (!b) { printf("  create FAILED\n"); continue; }
		printf("  create OK\n");

		int stride=0, size=0;
		printf("  === configure(640x480 x4) ===\n");
		int r = camhal_backend_configure(b, 640, 480, 4, &stride, &size);
		printf("  configure -> %d (stride=%d size=%d)\n", r, stride, size);

		printf("  === start ===\n");
		r = camhal_backend_start(b);
		printf("  start -> %d\n", r);

		printf("  === dqbuf ===\n");
		static uint8_t buf[640*480*3/2];
		uint64_t ts=0;
		r = camhal_backend_dqbuf(b, buf, sizeof(buf), &ts);
		printf("  dqbuf -> %d (ts=%lu)\n", r, (unsigned long)ts);

		camhal_backend_destroy(b);
		printf("  destroy OK\n");
	}
	camhal_free_cameras(cams, count);
	return 0;
}
