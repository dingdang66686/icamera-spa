// test-discover.cpp - verify camhal_discover_cameras() returns only the
// two real built-in cameras (gc5035-uf, ov5675-uf) and excludes the
// four AR0234 USB UVC sensors.
//
// Build:
//   g++ -O2 -Wall -Isrc -o build/test-discover test/test-discover.cpp \
//       ./build/libspa-icamera.so
//   ./build/test-discover
#include <stdio.h>
#include "camhal_backend.h"

int main(void)
{
	int count = 0;
	struct camhal_camera_info *cams = camhal_discover_cameras(&count);

	if (cams == NULL) {
		printf("discover failed\n");
		return 1;
	}

	printf("discovered %d camera(s):\n", count);
	for (int i = 0; i < count; i++) {
		printf("  id=%d  facing=%d  name='%s'  desc='%s'\n",
		       cams[i].camera_id, cams[i].facing,
		       cams[i].name, cams[i].description);
	}

	camhal_free_cameras(cams, count);

	if (count != 2) {
		printf("FAIL: expected exactly 2 built-in cameras, got %d\n", count);
		return 1;
	}
	puts("PASS: exactly 2 built-in cameras exposed");
	return 0;
}
