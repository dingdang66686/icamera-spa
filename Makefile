# icamera-spa : PipeWire SPA source plugin (libcamhal backend)
#
# Builds a SPA plugin that pulls NV12 frames directly from the Intel IPU6
# Camera HAL (libcamhal), bypassing GStreamer and v4l2loopback.
#
# The plugin is split into:
#   - src/icamera-source.c   : SPA node data plane + factory (plain C, gcc)
#   - src/camhal_backend.cpp : libcamhal capture backend     (C++, g++)
#                              because libcamhal is a C++ API.

CC       ?= gcc
CXX      ?= g++
PREFIX   ?= /usr
LIBDIR   ?= $(PREFIX)/lib

SPA_INCDIR   := $(shell pkg-config --cflags libpipewire-0.3 2>/dev/null)

CFLAGS  += -O2 -g -fPIC -Wall -Wextra $(SPA_INCDIR) -Isrc
CXXFLAGS += -O2 -g -fPIC -Wall -Wextra $(SPA_INCDIR) -Isrc
LDLIBS   += -lcamhal -lpthread

OBJ := build/icamera-source.o build/camhal_backend.o
SO  := build/libspa-icamera.so

all: $(SO) build/camhal-list build/camhal.so

$(SO): $(OBJ)
	$(CXX) -shared -o $@ $^ $(LDLIBS)

# Lua C module exposing CamHAL discovery to WirePlumber's enumerate-device.lua.
# WirePlumber's Lua sandbox blocks io.popen/os.execute, so instead of spawning
# the camhal-list helper we load this module through package.cpath
# (/usr/lib/lua/5.5/camhal.so).  Compiled as C++ because it talks to libcamhal's
# C++ API directly (like camhal-list.cpp); lua_* symbols keep C linkage via the
# extern "C" lua.h include.
build/camhal.so: src/camhal_lua.cpp
	@mkdir -p build
	$(CXX) -shared -fPIC -O2 -Wall -Wextra -I/usr/include -Isrc \
		-o $@ $< -lcamhal -llua -Wl,-soname,camhal.so

# Standalone camera discovery helper used by the WirePlumber monitor to
# enumerate camera nodes dynamically (see monitors/icamera/enumerate-device.lua).
build/camhal-list: src/camhal-list.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $< -lcamhal

build/icamera-source.o: src/icamera-source.c src/camhal_backend.h
	@mkdir -p build
	$(CC) $(CFLAGS) -c -o $@ $<

build/camhal_backend.o: src/camhal_backend.cpp src/camhal_backend.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Standalone HAL open/release cycle test (verify destroy releases the camera so
# a second open/open cycle succeeds, isolated from WirePlumber auto-linking).
build/test-hal-release: test/test-hal-release.cpp build/camhal_backend.o
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -std=c++11 -o $@ test/test-hal-release.cpp build/camhal_backend.o -lcamhal -lpthread

.PHONY: clean install install-monitor test-hal-release
test-hal-release: build/test-hal-release
clean:
	rm -rf build

# Install the SPA plugin only (the SPA data plane).  For the full WirePlumber
# integration (monitor scripts + drop-in config), call "make install-monitor".
install: $(SO)
	install -d $(LIBDIR)/spa-0.2/icamera
	install -m 755 $(SO) $(LIBDIR)/spa-0.2/icamera/libspa-icamera.so

# Install the WirePlumber monitor + Lua camhal discovery module + drop-in
# config so the cameras appear as PipeWire Video/Source nodes automatically.
#   - Lua C module -> /usr/lib/lua/<ver>/camhal.so (built here by 'all')
#   - monitor scripts -> /usr/share/wireplumber/scripts/monitors/icamera/
#   - drop-in config -> /usr/share/wireplumber/wireplumber.conf.d/51-icamera.conf
# The Lua ABI version is auto-detected for common distros if not overridden.
LUA_VER ?= $(shell ls /usr/lib/lua | sort -V | tail -1 2>/dev/null)
WP_MONITOR_DIR ?= /usr/share/wireplumber/scripts/monitors/icamera
WP_CONF_DIR    ?= /usr/share/wireplumber/wireplumber.conf.d

install-monitor: build/camhal.so
	install -d $(LIBDIR)/lua/$(LUA_VER)
	install -m 755 build/camhal.so $(LIBDIR)/lua/$(LUA_VER)/camhal.so
	install -d $(WP_MONITOR_DIR)
	install -m 644 monitors/icamera/enumerate-device.lua $(WP_MONITOR_DIR)/
	install -m 644 monitors/icamera/create-node.lua $(WP_MONITOR_DIR)/
	install -m 644 monitors/icamera/name-node.lua $(WP_MONITOR_DIR)/
	install -d $(WP_CONF_DIR)
	install -m 644 wireplumber/51-icamera.conf $(WP_CONF_DIR)/
	@echo "Remember to restart WirePlumber: systemctl --user restart wireplumber"
