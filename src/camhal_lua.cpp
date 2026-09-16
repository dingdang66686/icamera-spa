/*
 * camhal_lua.cpp - Lua (5.5) C module exposing CamHAL camera discovery.
 *
 * WirePlumber's Lua scripting sandbox strips io/os IPC (no io.popen, no
 * os.execute, only a restricted os.getenv), so the icamera monitor's
 * enumerate-device.lua cannot spawn a helper binary.  Instead we ship this
 * tiny Lua C module: enumerate-device.lua does `local camhal = require("camhal")`
 * and calls camhal.discover() to enumerate the built-in cameras.
 *
 * Discovery uses the HAL's get_number_of_cameras()/get_camera_info() which,
 * per the HAL docs, do NOT require camera_hal_init() - so it is safe to run
 * from the WirePlumber Lua sandbox.  USB / UVC raw sensors are excluded,
 * exactly like camhal-list.cpp and the SPA plugin's discovery.
 *
 * Build (Lua 5.5, matches WirePlumber):
 *   g++ -shared -fPIC -O2 -Isrc -o camhal.so src/camhal_lua.cpp -lcamhal
 * Install to /usr/lib/lua/5.5/camhal.so (WirePlumber package.cpath).
 *
 * SPDX-License-Identifier: MIT
 */
#include <cstring>

extern "C" {
#include <lua.h>
#include <lauxlib.h>
}

#include <libcamhal/api/ICamera.h>

#include "camhal_loader.h"

using namespace icamera;

/* Redirect the two discovery entry points to the on-demand loader table so the
 * HAL is dlopen()ed only while discover() runs and dlclose()d afterwards (its
 * destructor then removes the process-wide SysV shared-memory segment, which
 * would otherwise block camera access from a different UID). */
#define get_number_of_cameras(...) (icamera_loader::fn().get_number_of_cameras(__VA_ARGS__))
#define get_camera_info(...)       (icamera_loader::fn().get_camera_info(__VA_ARGS__))

static bool name_is_usb(const char *name)
{
	if (!name)
		return false;
	if (strstr(name, "usb") || strstr(name, "USB") ||
	    strstr(name, "uvc") || strstr(name, "UVC"))
		return true;
	return false;
}

/* camhal.discover() -> { {name=, facing=, camera_id=, description=}, ... } */
static int camhal_discover(lua_State *L)
{
        /* Load libcamhal just for this scan; unload it again on return. */
        icamera_loader::guard g;
        if (!g.ok()) {
                lua_createtable(L, 0, 0);
                return 1;
        }

	int total = get_number_of_cameras();
	lua_createtable(L, 0, 0);

	if (total <= 0)
		return 1;

	int n = 0;
	for (int id = 0; id < total; id++) {
		camera_info_t info;
		std::memset(&info, 0, sizeof(info));
		if (get_camera_info(id, info) < 0)
			continue;
		if (name_is_usb(info.name))
			continue;
		if (info.name == NULL || info.name[0] == '\0')
			continue;
		const char *desc = info.description && info.description[0]
				   ? info.description : info.name;

		lua_createtable(L, 0, 4);

		lua_pushinteger(L, id);
		lua_setfield(L, -2, "camera_id");

		lua_pushinteger(L, info.facing);
		lua_setfield(L, -2, "facing");

		lua_pushstring(L, info.name);
		lua_setfield(L, -2, "name");

		lua_pushstring(L, desc);
		lua_setfield(L, -2, "description");

		lua_rawseti(L, -2, ++n);
	}
	return 1;
}

static const luaL_Reg camhal_lib[] = {
	{ "discover", camhal_discover },
	{ NULL, NULL }
};

extern "C" int luaopen_camhal(lua_State *L)
{
	luaL_newlib(L, camhal_lib);
	return 1;
}
