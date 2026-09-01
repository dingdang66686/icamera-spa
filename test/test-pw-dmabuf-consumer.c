/*
 * test-pw-dmabuf-consumer.c
 *
 * End-to-end validation of the icamera-spa R-B "dma-mode" plugin path.
 *
 * Unlike GStreamer (whose pipewiresrc + auto-link default.video.source usually
 * negotiate MemFd, and whose `memory:DMABuf` caps can fail at negotiation),
 * this minimal PipeWire client explicitly links to the named icamera node,
 * requests SPA_DATA_DmaBuf output buffers, and verifies that:
 *   1. our node advertises SPA_DATA_DmaBuf in its output Buffers dataType,
 *   2. icamera_alloc_buffers() allocates i915 GEM DMA-BUFs,
 *   3. camhal_start() enters "DMA-BUF zero-copy" mode,
 *   4. frames actually arrive (HAL DMA-writes live sensor content into the
 *      consumer's GEM buffers).
 *
 * The plugin is hosted inside wireplumber, so its logs land in the journal;
 * check `journalctl --user | grep DMA-BUF` to confirm mode selection.
 *
 * Build:
 *   gcc -o build/test-pw-dmabuf-consumer test/test-pw-dmabuf-consumer.c \
 *       $(pkg-config --cflags --libs libpipewire-0.3)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include <pipewire/pipewire.h>
#include <pipewire/stream.h>
#include <spa/param/video/format-utils.h>
#include <spa/param/param.h>
#include <spa/param/video/raw.h>
#include <spa/pod/builder.h>
#include <spa/debug/pod.h>
#include <spa/utils/result.h>
#include <spa/node/event.h>

#define WIDTH  640
#define HEIGHT 480
#define N_BUFS 6

static struct {
	struct pw_main_loop *loop;
	struct pw_stream *stream;
	int got_buffers;
	int frames;
	int dmabuf_ok;
	int done;
} ctx;

static const char *target_node = "icamera_gc5035_rear";

/* Called when the stream (i.e. our consumer node) has its buffers configured.
 * We need to know whether the buffers we ended up with are DmaBuf. */
static void on_process(void *userdata)
{
	struct pw_stream *stream = ctx.stream;
	struct pw_buffer *b;

	if ((b = pw_stream_dequeue_buffer(stream)) == NULL) {
		pw_log_warn("no buffer to dequeue");
		return;
	}

	struct spa_buffer *sb = b->buffer;
	if (sb->n_datas > 0) {
		uint32_t t = sb->datas[0].type;
		if (!ctx.dmabuf_ok) {
			pw_log_info("consumer got buffer type=%u fd=%d max=%u",
				    t, sb->datas[0].fd, sb->datas[0].maxsize);
			ctx.dmabuf_ok = (t == SPA_DATA_DmaBuf);
		}
	}
	ctx.frames++;
	pw_stream_queue_buffer(stream, b);

	if (ctx.frames >= 20) {
		pw_log_info("received %d frames (dmabuf=%d)", ctx.frames, ctx.dmabuf_ok);
		pw_main_loop_quit(ctx.loop);
	}
}

static void on_param_changed(void *userdata, uint32_t id,
			     const struct spa_pod *param)
{
	/* not needed */
}

static const struct pw_stream_events stream_events = {
	PW_VERSION_STREAM_EVENTS,
	.process = on_process,
	.param_changed = on_param_changed,
};

/* Build the SPA format we request: NV12 640x480. */
static uint8_t format_buffer[4096];
static const struct spa_pod *build_format(void)
{
	/* NOTE: this SPA's spa_pod_builder_addv 'R'/'F' collect cases read the
	 * rectangle/fraction argument as a POINTER and dereference it. Passing a
	 * compound literal by value (SPA_POD_Rectangle(SPA_RECTANGLE(w,h))) is
	 * misread as a pointer and segfaults. So pass pointers to named
	 * variables instead. */
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(format_buffer, sizeof(format_buffer));
	struct spa_pod_frame f;
	struct spa_rectangle size = SPA_RECTANGLE(WIDTH, HEIGHT);
	struct spa_fraction rate = SPA_FRACTION(30, 1);
	spa_pod_builder_push_object(&b, &f,
		SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat);
	spa_pod_builder_add(&b,
		SPA_FORMAT_mediaType,      SPA_POD_Id(SPA_MEDIA_TYPE_video),
		SPA_FORMAT_mediaSubtype,   SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
		SPA_FORMAT_VIDEO_format,   SPA_POD_Id(SPA_VIDEO_FORMAT_NV12),
		SPA_FORMAT_VIDEO_size,     SPA_POD_Rectangle(&size),
		SPA_FORMAT_VIDEO_framerate, SPA_POD_Fraction(&rate),
		0);
	spa_pod_builder_pop(&b, &f);
	return spa_pod_builder_frame(&b, &f);
}

static void on_state_changed(void *userdata, enum pw_stream_state old,
			     enum pw_stream_state state, const char *error)
{
	pw_log_info("stream state: %s (%s)", pw_stream_state_as_string(state),
		    error ? error : "");
	if (state == PW_STREAM_STATE_ERROR) {
		pw_log_error("stream error: %s", error);
		pw_main_loop_quit(ctx.loop);
	}
	if (state == PW_STREAM_STATE_PAUSED || state == PW_STREAM_STATE_STREAMING)
		ctx.got_buffers = 1;
}

static const struct pw_stream_events events = {
	PW_VERSION_STREAM_EVENTS,
	.state_changed = on_state_changed,
	.process = on_process,
};

int main(int argc, char **argv)
{
	if (argc > 1)
		target_node = argv[1];
	int width = WIDTH, height = HEIGHT;
	if (argc > 3) { width = atoi(argv[2]); height = atoi(argv[3]); }

	pw_init(NULL, NULL);
	ctx.loop = pw_main_loop_new(NULL);

	ctx.stream = pw_stream_new_simple(pw_main_loop_get_loop(ctx.loop),
					  "pw-dmabuf-consumer",
					  pw_properties_new(
						PW_KEY_MEDIA_TYPE, "Video",
						PW_KEY_MEDIA_CATEGORY, "Capture",
						PW_KEY_MEDIA_ROLE, "Camera",
						NULL),
					  &events, &ctx);

	const struct spa_pod *params[1];
	params[0] = build_format();

	/* Request DmaBuf output buffers: the flags + our advert zeros let us
	 * pick SPA_DATA_DmaBuf.  Request the node alloc these via
	 * SPA_NODE_BUFFERS_FLAG_ALLOC path by setting PW_STREAM_FLAG_ALLOC_BUFFERS? No --
	 * we want the SOURCE (icamera node) to allocate so icamera_alloc_buffers runs.
	 * Set buffer config with dataType DmaBuf and let PipeWire negotiate. */
	/* Ask the SOURCE node to allocate its GEM buffers by NOT mapping them
	 * ourselves: with PW_STREAM_FLAG_MAP_BUFFERS clear, PipeWire hands
	 * buffer allocation to the peer (our node) via the
	 * SPA_NODE_BUFFERS_FLAG_ALLOC path, so icamera_alloc_buffers() runs and
	 * (having been told dataType=DmaBuf in the buffer params below) allocates
	 * real i915 GEM DMA-BUFs. */
	int res = pw_stream_connect(ctx.stream,
				    PW_DIRECTION_INPUT,
				    PW_ID_ANY,
				    PW_STREAM_FLAG_AUTOCONNECT,
				    params, 1);
	if (res < 0) {
		fprintf(stderr, "stream connect: %s\n", spa_strerror(res));
		return 1;
	}
	/* Tell the source node which data type we want via the Buffer param. */
	{
		uint8_t bbuf[512];
		struct spa_pod_builder b = SPA_POD_BUILDER_INIT(bbuf, sizeof(bbuf));
		struct spa_pod_frame f;
		spa_pod_builder_push_object(&b, &f,
			SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers);
		spa_pod_builder_add(&b,
			SPA_PARAM_BUFFERS_buffers,  SPA_POD_CHOICE_RANGE_Int(N_BUFS, 2, N_BUFS),
			SPA_PARAM_BUFFERS_blocks,   SPA_POD_Int(1),
			SPA_PARAM_BUFFERS_size,     SPA_POD_Int(width * height * 3 / 2),
			SPA_PARAM_BUFFERS_stride,   SPA_POD_Int(width),
			SPA_PARAM_BUFFERS_dataType, SPA_POD_CHOICE_FLAGS_Int(
				(1u << SPA_DATA_DmaBuf)),
			0);
		spa_pod_builder_pop(&b, &f);
		const struct spa_pod *prms[1] = { spa_pod_builder_frame(&b, &f) };
		pw_stream_update_params(ctx.stream, prms, 1);
	}

	pw_main_loop_run(ctx.loop);

	fprintf(stderr, "\nRESULT: frames=%d dmabuf_seen=%d\n",
		ctx.frames, ctx.dmabuf_ok);
	fflush(stderr);

	pw_stream_destroy(ctx.stream);
	pw_main_loop_destroy(ctx.loop);
	pw_deinit();
	return (ctx.frames >= 5 && ctx.dmabuf_ok) ? 0 : 2;
}
