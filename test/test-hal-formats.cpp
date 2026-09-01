/* test-hal-formats.cpp - dump every stream format+resolution the HAL
 * advertises for a given camera (via getSupportedStreamConfig), so we can
 * see which pixel formats beyond NV12 the IPU6 cameras really support.
 *
 * Usage: test-hal-formats [camera_id]   (default 4 = rear gc5035)
 */
#include <cstdio>
#include <cstring>
#include <libcamhal/api/ICamera.h>

using namespace icamera;

int main(int argc, char **argv)
{
	int camera_id = argc > 1 ? atoi(argv[1]) : 4;

	camera_info_t info;
	memset(&info, 0, sizeof(info));
	if (get_camera_info(camera_id, info) < 0 || !info.capability) {
		fprintf(stderr, "no camera %d\n", camera_id);
		return 1;
	}

	stream_array_t configs;
	configs.clear();
	info.capability->getSupportedStreamConfig(configs);

	printf("camera %d (%s) supports %zu stream configs:\n", camera_id,
	       info.name, configs.size());
	for (size_t i = 0; i < configs.size(); i++) {
		printf("  [%2zu] format=0x%08x '%c%c%c%c' %ux%u stride=%d size=%d\n",
		       i, configs[i].format,
		       (char)(configs[i].format & 0xff),
		       (char)((configs[i].format >> 8) & 0xff),
		       (char)((configs[i].format >> 16) & 0xff),
		       (char)((configs[i].format >> 24) & 0xff),
		       configs[i].width, configs[i].height,
		       configs[i].stride, configs[i].size);
	}
	return 0;
}
