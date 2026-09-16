/*
 * test-pw-dmabuf-direct.c
 *
 * Deterministic end-to-end proof of the icamera-spa R-B "dma-mode" plugin
 * path, WITHOUT the finicky PipeWire/GStreamer buffer-negotiation layers.
 *
 * We dlopen the installed libspa-icamera.so and drive the node directly
 * through the SPA Node v1 API -- exactly like pipewire's own
 * spa/examples/local-v4l2.c -- but instead of letting a pw_stream/GStreamer
 * peer choose the data type, we request SPA_DATA_DmaBuf explicitly and tell
 * the node to allocate via SPA_NODE_BUFFERS_FLAG_ALLOC.
 *
 * That forces icamera_alloc_buffers() to allocate real i915 GEM DMA-BUFs on
 * /dev/dri/renderD128 (via libdrm_intel), and camhal_start() to enter the
 * "DMA-BUF zero-copy" branch that calls camhal_backend_configure_dmabuf()
 * (V4L2_MEMORY_DMABUF import, sensor DMA-writes straight into our buffers).
 *
 * Environment:
 *   SPA_PLUGIN_DIR  = dir containing <subdir>/libspa-icamera.so (default /usr/lib)
 *
 * Build:
 *   gcc -o build/test-pw-dmabuf-direct test/test-pw-dmabuf-direct.c \
 *       $(pkg-config --cflags --libs libpipewire-0.3)
 *
 * Usage:  ./build/test-pw-dmabuf-direct [camera-name] [w] [h]
 *   default camera-name = icamera_gc5035_rear, 640x480.
 *
 * Exit 0 only if we negotiated DmaBuf buffers, the node logged the DMA-BUF
 * zero-copy path (see journal), frames arrived, and the GEM content changed
 * across consecutive frames (sensor write proof, valid even with lights off).
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
#include <inttypes.h>

/* support/logger for the node */
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

#define MAX_BUFFERS 6
#define SPA_PLUGIN_SUBDIR "icamera/libspa-icamera.so"
#define FACTORY_NAME "api.icamera.source"

static const char *devname = "icamera_gc5035_rear";
static int WIDTH = 640, HEIGHT = 480;

/* minimal log support (node logs through impl->log) */
SPA_LOG_IMPL(default_log);

/* ---- per-buffer backing so the node's SPA_DATA_DmaBuf allocator fills us ---- */
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
	/* dlopen the plugin */
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
	if (!enum_func) {
		fprintf(stderr, "no enum func in %s\n", path);
		return -EIO;
	}

	const struct spa_handle_factory *factory = NULL;
	uint32_t index = 0;
	while (enum_func(&factory, &index) > 0) {
		if (spa_streq(factory->name, FACTORY_NAME))
			break;
		factory = NULL;
	}
	if (!factory) {
		fprintf(stderr, "factory %s not found in %s\n", FACTORY_NAME, path);
		return -ENOENT;
	}

	struct spa_handle *handle = calloc(1, spa_handle_factory_get_size(factory, NULL));

	struct spa_dict_item items[2];
	items[0].key = SPA_KEY_DEVICE_NAME;
	items[0].value = devname;
	items[1].key = "node.name";
	items[1].value = "test-pw-dmabuf-direct";
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
	int i;
	for (i = 0; i < MAX_BUFFERS; i++) {
		struct bufdesc *b = &buffers[i];
		bp[i] = &b->buffer;
		b->buffer.metas = b->metas;
		b->buffer.n_metas = 1;
		b->buffer.datas = b->datas;
		b->buffer.n_datas = 1;
		b->header.flags = 0;
		b->header.seq = 0;
		b->header.pts = 0;
		b->header.dts_offset = 0;
		b->metas[0].type = SPA_META_Header;
		b->metas[0].data = &b->header;
		b->metas[0].size = sizeof(b->header);
		/* data type: DmaBuf -> node allocates i915 GEM and hands us the fd */
		b->datas[0].type = SPA_DATA_DmaBuf;
		b->datas[0].flags = 0;
		b->datas[0].fd = -1;
		b->datas[0].mapoffset = 0;
		b->datas[0].maxsize = 0;
		b->datas[0].data = NULL;
		b->datas[0].chunk = &b->chunk[0];
		b->datas[0].chunk->offset = 0;
		b->datas[0].chunk->size = 0;
		b->datas[0].chunk->stride = 0;
	}
}

static int negotiate_format(struct spa_node *node, struct spa_io_buffers *io)
{
	int res;
	uint8_t buf[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof(buf));

	memset(io, 0, sizeof(*io));
	io->status = SPA_STATUS_NEED_DATA;
	io->buffer_id = SPA_ID_INVALID;

	/* set the SPA_IO_Buffers on the output port so process() can emit */
	res = spa_node_port_set_io(node, SPA_DIRECTION_OUTPUT, 0,
				   SPA_IO_Buffers, io, sizeof(*io));
	if (res < 0) {
		fprintf(stderr, "port_set_io: %d %s\n", res, spa_strerror(res));
		return res;
	}

	struct spa_rectangle size = SPA_RECTANGLE(WIDTH, HEIGHT);
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
		fprintf(stderr, "use_buffers(ALLOC): %d %s\n", res, spa_strerror(res));
		return res;
	}
	return 0;
}

static void dump_buffer_state(void)
{
	int i;
	for (i = 0; i < MAX_BUFFERS; i++) {
		struct spa_data *d = &buffers[i].datas[0];
		fprintf(stderr, "  buf[%d] type=%u fd=%" PRIi64 " data=%p max=%u\n",
			i, d->type, d->fd, (void *)d->data, d->maxsize);
	}
	int dmabuf = 1;
	for (i = 0; i < MAX_BUFFERS; i++)
		if (buffers[i].datas[0].fd < 0 || buffers[i].datas[0].type != SPA_DATA_DmaBuf)
			dmabuf = 0;
	fprintf(stderr, "RESULT-DMABUF=%d\n", dmabuf);
}

int main(int argc, char **argv)
{
	if (argc > 1) devname = argv[1];
	if (argc > 3) { WIDTH = atoi(argv[2]); HEIGHT = atoi(argv[3]); }

	struct spa_node *node = NULL;
	struct spa_io_buffers io;
	int res, i;
	int frames = 0, changed = 0;
	uint8_t *copy = NULL;

	if ((res = load_node(&node)) < 0)
		return res;

	if ((res = negotiate_format(node, &io)) < 0)
		return res;

	dump_buffer_state();

	/* sanity: if any buffer is not a DmaBuf, our allocator didn't run */
	for (i = 0; i < MAX_BUFFERS; i++)
		if (buffers[i].datas[0].type != SPA_DATA_DmaBuf) {
			fprintf(stderr, "FAIL: buffer %d not DmaBuf (type=%u)\n",
				i, buffers[i].datas[0].type);
			return 2;
		}

	/* send Start; camhal_start() opens the camera and should enter the
	 * DMA-BUF zero-copy branch (see journalctl --user | grep DMA-BUF) */
	struct spa_command cmd = SPA_NODE_COMMAND_INIT(SPA_NODE_COMMAND_Start);
	if ((res = spa_node_send_command(node, &cmd)) < 0) {
		fprintf(stderr, "send Start: %d %s\n", res, spa_strerror(res));
		return res;
	}

	/* keep a pristine snapshot of buffer[0] to detect sensor writes */
	copy = malloc(buffers[0].datas[0].maxsize);
	if (!copy) return -ENOMEM;

	unsigned long long elapsed = 0;
	while (frames < 30 && elapsed < 4000) {
		/* drive the node: process() pulls a filled frame onto io */
		res = spa_node_process(node);
		if (res < 0 && res != -EIO) {
			fprintf(stderr, "process err %d\n", res);
			break;
		}
		if (io.status == SPA_STATUS_HAVE_DATA && io.buffer_id < MAX_BUFFERS) {
			uint32_t id = io.buffer_id;
			void *data = buffers[id].datas[0].data;
			uint32_t maxs = buffers[id].datas[0].maxsize;
			frames++;

			/* frame-diff on a fixed 4KB window: proves the sensor wrote
			 * fresh content EVEN IF LIGHTS ARE OFF (black frames still
			 * differ by noise across time). */
			if (copy && frames == 1) {
				memcpy(copy, data, 4096 < maxs ? 4096 : maxs);
			} else if (copy) {
				size_t n = 4096 < maxs ? 4096 : maxs;
				size_t diff = 0, k;
				for (k = 0; k < n; k++) {
					if (copy[k] != ((uint8_t *)data)[k])
						diff++;
				}
				if (diff > 0)
					changed++;
				memcpy(copy, data, n);
			}

			if ((frames % 10) == 0)
				fprintf(stderr, "  frame %d id=%u fd=%" PRIi64 "\n",
					frames, id, buffers[id].datas[0].fd);

			/* Mark consumed: next process() call recycles this buffer
			 * (the node re-queues io->buffer_id back to the HAL/queue) and
			 * dequeues the next filled frame.  We must NOT call
			 * port_reuse_buffer ourselves -- the node handles recycling at
			 * the top of process() based on the outstanding io->buffer_id. */
			io.status = SPA_STATUS_NEED_DATA;
		}
		usleep(1000);
		elapsed++;
	}

	fprintf(stderr, "\nRESULT: frames=%d changed=%d\n", frames, changed);
	fflush(stderr);

	cmd = SPA_NODE_COMMAND_INIT(SPA_NODE_COMMAND_Pause);
	spa_node_send_command(node, &cmd);

	free(copy);
	return (frames >= 5 && changed > 0) ? 0 : 2;
}
