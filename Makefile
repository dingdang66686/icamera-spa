# icamera-spa : PipeWire SPA source plugin (libcamhal backend)
#
# Builds a SPA plugin that pulls NV12 frames directly from the Intel IPU6
# Camera HAL (libcamhal), bypassing GStreamer and v4l2loopback.
#
# The plugin is split into:
#   - src/icamera-source.c    : SPA node data plane + factory (plain C, gcc)
#   - src/icamera-format.c    : V4L2 fourcc / SPA format + fps helpers (pure)
#   - src/icamera-metadata.c  : per-frame 3A SPA_META_Control writer (pure)
#   - src/camhal_backend.cpp  : libcamhal capture backend     (C++, g++)
#                              because libcamhal is a C++ API.

CC       ?= gcc
CXX      ?= g++
PREFIX   ?= /usr
LIBDIR   ?= $(PREFIX)/lib

SPA_INCDIR   := $(shell pkg-config --cflags libpipewire-0.3 2>/dev/null)
# Pull in the PipeWire/SPA headers as *system* headers (-isystem instead of
# -I).  -Wextra otherwise reports two warnings that originate inside them and
# that we cannot fix:
#   spa/param/param.h:66     missing initializer for 'user' in spa_param_info
#   spa/pod/compare.h:190    unused parameter 'size'
# Our own sources keep the full -Wall -Wextra treatment.
SPA_INCDIR   := $(patsubst -I%,-isystem %,$(SPA_INCDIR))
# libpipewire itself is only needed by the pw_stream-based tests
# (test-pw-dmabuf-consumer); the SPA tests dlopen the plugin instead.
PW_LIBS      := $(shell pkg-config --libs libpipewire-0.3 2>/dev/null)

CFLAGS  += -O2 -g -fPIC -Wall -Wextra $(SPA_INCDIR) -Isrc
CXXFLAGS += -O2 -g -fPIC -Wall -Wextra $(SPA_INCDIR) -Isrc
# libcamhal is NOT linked directly any more: the backend loads it on demand via
# dlopen()/dlsym() (see src/camhal_loader.h) so the HAL - and the process-wide
# SysV shared-memory segment it creates on load - only exists while a camera is
# actually open.  Hence -ldl instead of -lcamhal.
LDLIBS   += -lpthread -ldl

# R-B dma-mode compile-time switch (default ON).
#
#   make                      -> ENABLE_DMA_BUF=1, links libdrm_intel/libdrm
#   make ENABLE_DMA_BUF=0     -> compiles out the whole DMA-BUF path and does
#                                NOT link libdrm_intel/libdrm (plugin only ever
#                                advertises/uses MemFd)
#
# src/icamera-source.c and src/camhal_backend.* default ENABLE_DMA_BUF to 1 if
# unspecified; we set it explicitly here so the linkage matches the code.
ENABLE_DMA_BUF ?= 1
ifeq ($(ENABLE_DMA_BUF),0)
CFLAGS  += -DENABLE_DMA_BUF=0
CXXFLAGS += -DENABLE_DMA_BUF=0
else
CFLAGS  += -DENABLE_DMA_BUF=1
CXXFLAGS += -DENABLE_DMA_BUF=1
LDLIBS  += -ldrm_intel -ldrm
endif

OBJ := build/icamera-source.o build/icamera-format.o \
       build/icamera-metadata.o build/camhal_backend.o
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
build/camhal.so: src/camhal_lua.cpp src/camhal_loader.h
	@mkdir -p build
	$(CXX) -shared -fPIC -O2 -Wall -Wextra -I/usr/include -Isrc \
		-o $@ $< -llua -ldl -Wl,-soname,camhal.so

# Standalone camera discovery helper used by the WirePlumber monitor to
# enumerate camera nodes dynamically (see monitors/icamera/enumerate-device.lua).
# libcamhal is dlopen()ed on demand (see src/camhal_loader.h).
build/camhal-list: src/camhal-list.cpp src/camhal_loader.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $< -ldl

build/icamera-source.o: src/icamera-source.c src/camhal_backend.h \
		src/icamera-format.h src/icamera-metadata.h
	@mkdir -p build
	$(CC) $(CFLAGS) -c -o $@ $<

build/icamera-format.o: src/icamera-format.c src/icamera-format.h
	@mkdir -p build
	$(CC) $(CFLAGS) -c -o $@ $<

build/icamera-metadata.o: src/icamera-metadata.c src/icamera-metadata.h \
		src/camhal_backend.h
	@mkdir -p build
	$(CC) $(CFLAGS) -c -o $@ $<

build/camhal_backend.o: src/camhal_backend.cpp src/camhal_backend.h \
		src/camhal_loader.h src/camhal_busy.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Standalone HAL open/release cycle test (verify destroy releases the camera so
# a second open/open cycle succeeds, isolated from WirePlumber auto-linking).
build/test-hal-release: test/test-hal-release.cpp build/camhal_backend.o
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -std=c++11 -o $@ test/test-hal-release.cpp build/camhal_backend.o -lpthread -ldl

# Standalone R-B "dma-mode" test: allocate real i915 GEM DMA-BUFs, configure
# the backend with camhal_backend_configure_dmabuf() (V4L2_MEMORY_DMABUF import),
# stream via dqbuf_index/release and confirm the HAL writes live content into
# our buffers.  Triggers on libdrm_intel for /dev/dri/renderD128 GEM export.
build/test-hal-dmabuf: test/test-hal-dmabuf.cpp build/camhal_backend.o
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -std=c++11 -o $@ test/test-hal-dmabuf.cpp build/camhal_backend.o -lpthread -ldl -ldrm_intel -ldrm

.PHONY: clean install install-monitor test-hal-release test-hal-dmabuf \
	test-pw-dmabuf-direct test-pw-dmabuf-consumer test-pw-reneg test-pw-props
test-hal-release: build/test-hal-release
test-hal-dmabuf: build/test-hal-dmabuf

# End-to-end R-B dma-mode proof: a minimal direct SPA driver that dlopens the
# installed libspa-icamera.so and forces DmaBuf allocation
# (SPA_DATA_DmaBuf + SPA_NODE_BUFFERS_FLAG_ALLOC), then Start + process loop.
build/test-pw-dmabuf-direct: test/test-pw-dmabuf-direct.c
	@mkdir -p build
	$(CC) -O2 -g -Wall -Wextra -o $@ $< $(SPA_INCDIR)

# pw_stream-based DmaBuf consumer (negotiates MemFd only; kept for reference).
build/test-pw-dmabuf-consumer: test/test-pw-dmabuf-consumer.c
	@mkdir -p build
	$(CC) -O2 -g -Wall -Wextra -o $@ $< $(SPA_INCDIR) $(PW_LIBS)

test-pw-dmabuf-direct: build/test-pw-dmabuf-direct
test-pw-dmabuf-consumer: build/test-pw-dmabuf-consumer

# Renegotiate regression test: dlopens the installed libspa-icamera.so and
# drives the live node through several resolution changes (R-B dma-mode),
# validating the icamera_clear_buffers() use-after-free / buffer-pool rebuild
# fix.  Exit 0 only if every step survives and frames keep flowing.
build/test-pw-reneg: test/test-pw-reneg.c
	@mkdir -p build
	$(CC) -O2 -g -Wall -Wextra -o $@ $< $(SPA_INCDIR)

test-pw-reneg: build/test-pw-reneg

# SPA Props/PropInfo contract test: dlopens the installed libspa-icamera.so and
# asserts every advertised property id lives in a legal namespace, is wrapped
# in a Choice pod, carries a description (and labels for enums), is advertised
# identically on the node and port channels, appears in SPA_PARAM_Props, and
# survives a set_param() round-trip.
build/test-pw-props: test/test-pw-props.c
	@mkdir -p build
	$(CC) -O2 -g -Wall -Wextra -o $@ $< $(SPA_INCDIR)

test-pw-props: build/test-pw-props
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
