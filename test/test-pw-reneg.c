/*
 * test-pw-reneg.c
 *
 * Programmatic regression test for the icamera renegotiate use-after-free
 * fix (see icamera-source.c: icamera_clear_buffers / struct frame own_*).
 *
 * Root cause being validated:
 *   Selecting a different resolution/framerate in OBS causes a runtime
 *   renegotiate on the live icamera node.  Previously icamera_clear_buffers()
 *   freed the OWNED backing memory by dereferencing frame->outbuf->datas[0],
 *   but during a renegotiate PipeWire may already have freed that spa_buffer,
 *   so reading through the stale pointer was a use-after-free that crashed
 *   WirePlumber (and, in turn, took OBS down with it).
 *
 * This driver renegotiates the live node to several sizes in a row (with the
 * capture thread running) and aborts/crashes if that path is still broken.
 * It drives the node through the SPA Node v1 API exactly like
 * test-pw-dmabuf-direct.c (dlopen the installed libspa-icamera.so, R-B
 * dma-mode with SPA_NODE_BUFFERS_FLAG_ALLOC).
 *
 * Build:
 *   gcc -O2 -g -Wall -o build/test-pw-reneg test/test-pw-reneg.c \
 *       $(pkg-config --cflags --libs libpipewire-0.3)
 *
 * Usage: ./build/test-pw-reneg [camera-name]
 * Exit 0 only if every renegotiate step survived and frames kept flowing.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>

#include <spa/support/log.h>
#include <spa/support/log-impl.h>
#include <spa/support/plugin.h>
#include <spa/support/loop.h>
#include <spa/utils/names.h>
#include <spa/utils/result.h>
#include <spa/utils/string.h>
#include <spa/monitor/device.h>
#include <spa/node/node.h>
#include <spa/node/io.h>
#include <spa/node/utils.h>
#include <spa/param/param.h>
#include <spa/param/props.h>
#include <spa/param/video/format-utils.h>
#include <spa/param/video/raw.h>
#include <spa/debug/pod.h>

#define MAX_BUFFERS 8
#define SPA_PLUGIN_SUBDIR "icamera/libspa-icamera.so"
#define FACTORY_NAME "api.icamera.source"

static const char *devname = "icamera_gc5035_rear";

SPA_LOG_IMPL(default_log);

struct bufdesc {
	struct spa_buffer buffer;
	struct spa_meta metas[2];
	struct spa_meta_header header;
	struct spa_data datas[1];
	struct spa_chunk chunk[1];
};

static struct bufdesc buffers[MAX_BUFFERS];
static struct spa_buffer *bp[MAX_BUFFERS];

static int load_node(struct spa_node **node)
{
	char path[512];
	snprintf(path, sizeof(path), "%s/%s",
		 getenv("SPA_PLUGIN_DIR") ? getenv("SPA_PLUGIN_DIR") : "/usr/lib",
		 SPA_PLUGIN_SUBDIR);
	void *hnd = dlopen(path, RTLD_NOW);
	if (!hnd) {
		fprintf(stderr, "dlopen %s: %s\n", path, dlerror());
		return -EIO;
	}
	spa_handle_factory_enum_func_t enum_func =
		(spa_handle_factory_enum_func_t)dlsym(hnd, SPA_HANDLE_FACTORY_ENUM_FUNC_NAME);
	if (!enum_func)
		return -EIO;

	const struct spa_handle_factory *factory = NULL;
	uint32_t index = 0;
	while (enum_func(&factory, &index) > 0) {
		if (spa_streq(factory->name, FACTORY_NAME))
			break;
		factory = NULL;
	}
	if (!factory)
		return -ENOENT;

	struct spa_handle *handle = calloc(1, spa_handle_factory_get_size(factory, NULL));
	struct spa_dict_item items[2];
	items[0].key = SPA_KEY_DEVICE_NAME;
	items[0].value = devname;
	items[1].key = "node.name";
	items[1].value = "test-pw-reneg";
	struct spa_dict info = SPA_DICT_INIT_ARRAY(items);

	struct spa_support support[2];
	support[0] = SPA_SUPPORT_INIT(SPA_TYPE_INTERFACE_Log, &default_log.log);
	uint32_t n_support = 1;

	int res = spa_handle_factory_init(factory, handle, &info, support, n_support);
	if (res < 0) {
		fprintf(stderr, "factory init: %d %s\n", res, spa_strerror(res));
		return res;
	}
	res = spa_handle_get_interface(handle, SPA_TYPE_INTERFACE_Node, (void **)node);
	if (res < 0) {
		fprintf(stderr, "get Node interface: %s\n", spa_strerror(res));
		return res;
	}
	return 0;
}

static void setup_buffers(void)
{
	memset(buffers, 0, sizeof(buffers));
	for (int i = 0; i < MAX_BUFFERS; i++) {
		struct bufdesc *b = &buffers[i];
		b->buffer.n_datas = 1;
		b->buffer.datas = b->datas;
		b->datas[0].type = SPA_DATA_DmaBuf; /* request dma-mode allocator */
		b->datas[0].fd = -1;
		b->buffer.n_metas = 2;
		b->buffer.metas = b->metas;
		b->metas[0].type = SPA_META_Header;
		b->metas[0].data = &b->header;
		b->metas[0].size = sizeof(b->header);
		b->datas[0].chunk = &b->chunk[0];
		bp[i] = &b->buffer;
	}
}

static int negotiate_format(struct spa_node *node, struct spa_io_buffers *io,
			    int width, int height)
{
	int res;
	uint8_t buf[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof(buf));

	memset(io, 0, sizeof(*io));
	io->status = SPA_STATUS_NEED_DATA;
	io->buffer_id = SPA_ID_INVALID;

	res = spa_node_port_set_io(node, SPA_DIRECTION_OUTPUT, 0,
				   SPA_IO_Buffers, io, sizeof(*io));
	if (res < 0) {
		fprintf(stderr, "port_set_io: %d %s\n", res, spa_strerror(res));
		return res;
	}

	struct spa_rectangle size = SPA_RECTANGLE(width, height);
	struct spa_fraction rate = SPA_FRACTION(30, 1);
	struct spa_pod *format;
	format = spa_format_video_raw_build(&b, 0,
			&SPA_VIDEO_INFO_RAW_INIT(
				.format = SPA_VIDEO_FORMAT_NV12,
				.size = size,
				.framerate = rate));

	res = spa_node_port_set_param(node, SPA_DIRECTION_OUTPUT, 0,
				      SPA_PARAM_Format, 0, format);
	if (res < 0) {
		fprintf(stderr, "port_set_param format: %d %s\n", res, spa_strerror(res));
		return res;
	}
	setup_buffers();

	res = spa_node_port_use_buffers(node, SPA_DIRECTION_OUTPUT, 0,
					SPA_NODE_BUFFERS_FLAG_ALLOC,
					bp, MAX_BUFFERS);
	if (res < 0) {
		fprintf(stderr, "use_buffers(ALLOC) %dx%d: %d %s\n",
			width, height, res, spa_strerror(res));
		return res;
	}
	return 0;
}

static int run_frames(struct spa_node *node, struct spa_io_buffers *io,
		      const char *stage, int nframes)
{
	int frames = 0;
	unsigned long long elapsed = 0;
	while (frames < nframes && elapsed < 3000) {
		int res = spa_node_process(node);
		if (res < 0 && res != -EIO) {
			fprintf(stderr, "  [%s] process err %d\n", stage, res);
			return -1;
		}
		if (io->status == SPA_STATUS_HAVE_DATA && io->buffer_id < MAX_BUFFERS) {
			frames++;
			io->status = SPA_STATUS_NEED_DATA;
		}
		usleep(1000);
		elapsed++;
	}
	fprintf(stderr, "  [%s] frames=%d\n", stage, frames);
	return frames;
}

int main(int argc, char **argv)
{
	if (argc > 1) devname = argv[1];

	struct spa_node *node = NULL;
	struct spa_io_buffers io;
	int res;

	if ((res = load_node(&node)) < 0)
		return res;

	fprintf(stderr, "stage 1: negotiate 2560x1920 + start\n");
	if ((res = negotiate_format(node, &io, 2560, 1920)) < 0)
		return res;

	struct spa_command cmd = SPA_NODE_COMMAND_INIT(SPA_NODE_COMMAND_Start);
	if ((res = spa_node_send_command(node, &cmd)) < 0) {
		fprintf(stderr, "send Start: %d %s\n", res, spa_strerror(res));
		return res;
	}

	if (run_frames(node, &io, "S1", 6) < 3) {
		fprintf(stderr, "FAIL: no frames after first start\n");
		return 2;
	}

	/* --- the actual regression: renegotiate on a live node --- */
	fprintf(stderr, "stage 2: renegotiate -> 2560x1440 (live)\n");
	if ((res = negotiate_format(node, &io, 2560, 1440)) < 0) {
		fprintf(stderr, "FAIL: renegotiate to 1440 failed: %d\n", res);
		return 3;
	}
	if (run_frames(node, &io, "S2", 6) < 3) {
		fprintf(stderr, "FAIL: no frames after renegotiate -> 1440\n");
		return 4;
	}

	fprintf(stderr, "stage 3: renegotiate -> 1280x720 (live)\n");
	if ((res = negotiate_format(node, &io, 1280, 720)) < 0) {
		fprintf(stderr, "FAIL: renegotiate to 720p failed: %d\n", res);
		return 5;
	}
	if (run_frames(node, &io, "S3", 6) < 3) {
		fprintf(stderr, "FAIL: no frames after renegotiate -> 720p\n");
		return 6;
	}

	fprintf(stderr, "stage 4: renegotiate -> back to 2560x1920 (live)\n");
	if ((res = negotiate_format(node, &io, 2560, 1920)) < 0) {
		fprintf(stderr, "FAIL: final renegotiate failed: %d\n", res);
		return 7;
	}
	if (run_frames(node, &io, "S4", 4) < 2) {
		fprintf(stderr, "FAIL: no frames after final renegotiate\n");
		return 8;
	}

	fprintf(stderr, "\nRESULT: all renegotiate stages survived\n");
	fflush(stderr);

	cmd = SPA_NODE_COMMAND_INIT(SPA_NODE_COMMAND_Pause);
	spa_node_send_command(node, &cmd);
	return 0;
}
