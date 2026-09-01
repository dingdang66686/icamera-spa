/*
 * camhal-list.cpp - standalone CamHAL camera discovery helper
 *
 * Prints the real built-in cameras known to the Intel IPU6 libcamhal,
 * one per line, tab-separated:
 *
 *     <name>\t<facing>\t<description>
 *
 * facing: 0 = rear, 1 = front.
 *
 * USB / UVC (AR0234 etc.) raw sensors are excluded, exactly like the
 * icamera SPA plugin's camhal_discover_cameras().
 *
 * The WirePlumber icamera monitor (enumerate-device.lua) calls this to
 * discover camera nodes DYNAMICALLY instead of hardcoding the camera
 * table.
 *
 * Build (in icamera-spa/):
 *   g++ -O2 -Wall -Isrc -o build/camhal-list src/camhal-list.cpp -lcamhal
 *
 * Uses the HAL's get_number_of_cameras()/get_camera_info() which, per the
 * HAL docs, do NOT require camera_hal_init() - so it is safe to run as a
 * one-shot helper at any time.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>

#include <libcamhal/api/ICamera.h>

using namespace icamera;

static bool name_is_usb(const char *name)
{
	if (!name)
		return false;
	if (strstr(name, "usb") || strstr(name, "USB") ||
	    strstr(name, "uvc") || strstr(name, "UVC"))
		return true;
	return false;
}

int main(void)
{
	int total = get_number_of_cameras();
	if (total <= 0)
		return 1;

	for (int id = 0; id < total; id++) {
		camera_info_t info;
		memset(&info, 0, sizeof(info));
		if (get_camera_info(id, info) < 0)
			continue;
		if (name_is_usb(info.name))
			continue;
		if (info.name == NULL || info.name[0] == '\0')
			continue;
		const char *desc = info.description && info.description[0]
				   ? info.description : info.name;
		printf("%s\t%d\t%s\n", info.name, info.facing, desc);
	}
	return 0;
}
