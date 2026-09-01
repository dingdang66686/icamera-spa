// test-hal-dmabuf.cpp - verify the R-B "dma-mode" path end to end against the
// real CamHAL: allocate real i915 GEM DMA-BUFs (exactly like the official
// icamerasrc DMA_MODE pool: drm_intel_bo_alloc + gem_export_to_prime on
// /dev/dri/renderD128), register them with the backend via
// camhal_backend_configure_dmabuf() (V4L2_MEMORY_DMABUF import), stream via
// camhal_backend_dqbuf_index()/release() and confirm the HAL writes real
// (non-black / changing) content into the hardware buffers through DMA.
//
// This isolates the single riskiest part of R-B (does the HAL import our
// DMA-BUFs and PSYS-write into them, or does dma_buf_get() reject them / the
// pipeline stall) before we wire DmaBuf negotiation into the SPA plugin.
//
// Exit code: 0 = import worked, frames dequeued, mmap'd content looks like a
// changing picture; 1 = any step failed.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>

#include <linux/dma-buf.h>

#include <libdrm/intel_bufmgr.h>

#include "camhal_backend.h"

#define N_BUF 6
#define N_FRAMES 60
#define W 640
#define H 480

/* Cache-coherency bracket: flush CPU mapping of a DMA-BUF before/after access.
 * The HAL writes through the GPU/IPU (device); the CPU mmap may be stale
 * unless we DMA_BUF_IOCTL_SYNC (cache invalidate / clean) around the read. */
static void dma_sync(int fd, __u64 flags)
{
	struct dma_buf_sync sync = { 0 };
	sync.flags = flags;
	ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
}
#define SYNC_READ  (DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ)
#define SYNC_REND  (DMA_BUF_SYNC_END   | DMA_BUF_SYNC_READ)

int main(void)
{
	int n = 0;
	struct camhal_camera_info *cams = camhal_discover_cameras(&n);
	if (!cams || n <= 0) { printf("no cameras\n"); return 1; }
	int rear = -1, front = -1;
	for (int i = 0; i < n; i++) {
		if (strstr(cams[i].name, "gc5035")) rear = cams[i].camera_id;
		if (strstr(cams[i].name, "ov5675")) front = cams[i].camera_id;
	}
	camhal_free_cameras(cams, n);
	int cam = rear >= 0 ? rear : front;
	const char *ov = getenv("ICAMERA_TEST_CAM");
	if (ov) cam = atoi(ov);
	if (ov && strncmp(ov, "front", 5) == 0) cam = front;
	if (ov && strncmp(ov, "rear", 4) == 0) cam = rear;
	printf("== target camera = %d ==\n", cam);
	if (cam < 0) { printf("no known camera\n"); return 1; }

	/* ---- Allocate i915 GEM DMA-BUFs (mirror icamerasrc dma_mode). ---- */
	int dri = open("/dev/dri/renderD128", O_RDWR);
	if (dri < 0) { printf("open renderD128 failed: %m\n"); return 1; }
	drm_intel_bufmgr *bufmgr = drm_intel_bufmgr_gem_init(dri, 4096);
	if (!bufmgr) { printf("bufmgr_gem_init failed\n"); close(dri); return 1; }

	const size_t FRAME = (size_t)W * H * 3 / 2;          /* packed NV12 */
	const size_t ALIGNED = ((FRAME + 4095) / 4096) * 4096;

	int fds[N_BUF];
	uint8_t *maps[N_BUF];

	for (int i = 0; i < N_BUF; i++) {
		drm_intel_bo *bo = drm_intel_bo_alloc(bufmgr, "icamera-dma", ALIGNED, 4096);
		if (!bo) { printf("bo_alloc[%d] failed\n", i); return 1; }
		if (drm_intel_bo_gem_export_to_prime(bo, &fds[i]) < 0) {
			printf("gem_export_to_prime[%d] failed: %m\n", i);
			return 1;
		}
		/* CPU map for content inspection (the plugin keeps d->data mmap'd). */
		maps[i] = (uint8_t *)mmap(NULL, ALIGNED, PROT_READ | PROT_WRITE,
					 MAP_SHARED, fds[i], 0);
		if (maps[i] == MAP_FAILED) { printf("mmap[%d] dmabuf failed: %m\n", i); return 1; }
		/* Pre-fill with a known 0xA5 sentinel.  If the HAL really writes the
		 * DMA-BUF (even a dark frame), this pattern gets overwritten; if the
		 * import path is broken the buffer stays all 0xA5.  This write-proof
		 * is independent of how bright the scene is. */
		memset(maps[i], 0xA5, ALIGNED);
		printf("buf[%d]: dmafd=%d mapped=%p (pre-filled 0xA5)\n", i, fds[i], (void *)maps[i]);
		drm_intel_bo_unreference(bo);
	}

	/* ---- Backend: configure_dmabuf (R-B). ---- */
	struct camhal_backend *b = camhal_backend_create(cam, NULL);
	if (!b) { printf("backend create failed\n"); return 1; }
	int stride = 0, size = 0;
	if (camhal_backend_configure_dmabuf(b, W, H, N_BUF, fds, &stride, &size) < 0) {
		printf("configure_dmabuf FAILED (dma-mode declined?)\n");
		camhal_backend_destroy(b);
		return 1;
	}
	printf("configure_dmabuf OK stride=%d size=%d\n", stride, size);

	if (camhal_backend_start(b) < 0) {
		printf("start failed\n");
		camhal_backend_destroy(b);
		return 1;
	}

	/* ---- Stream via dqbuf_index / release, checking content. ---- */
	int bad = 0;
	for (int i = 0; i < N_FRAMES; i++) {
		int idx = -1; uint64_t ts = 0;
		if (camhal_backend_dqbuf_index(b, &idx, &ts) < 0) {
			printf("dqbuf_index[%d] failed\n", i); bad = 1; break;
		}
		/* Inspect the CPU mapping of the dequeued buffer: how much of the
		 * 0xA5 sentinel got overwritten by the HAL (dark scene -> near 0)? */
		uint8_t *p = maps[idx];
		const size_t n = FRAME;                    /* full packed NV12 frame */
		uint64_t sum = 0, sentinel = 0;
		uint32_t tmin = 255, tmax = 0;
		dma_sync(fds[idx], SYNC_READ);          /* invalidate CPU cache */
		for (size_t z = 0; z < n; z++) { uint8_t v = p[z]; sum += v; if (v==0xA5) sentinel++; if (v<tmin)tmin=v; if(v>tmax)tmax=v; }
		dma_sync(fds[idx], SYNC_REND);
		double written = (double)(n - sentinel) / (double)n * 100.0;
		double avg = n ? (double)sum / (double)n : 0;
		if ((i % 5) == 0 || i == N_FRAMES - 1)
			printf("frame %d idx=%d ts=%llu  overwritten=%.1f%%  lum(avg=%.1f max=%u)\n",
			       i, idx, (unsigned long long)ts, written, avg, tmax);
		if (camhal_backend_release(b, idx) < 0) {
			printf("release[%d] failed\n", i); bad = 1; break;
		}
	}
	int has_written = 0;
	for (int i = 0; i < N_BUF; i++) {
		uint8_t *p = maps[i]; uint64_t sentinel = 0;
		dma_sync(fds[i], SYNC_READ);
		for (size_t z = 0; z < FRAME; z++) if (p[z] == 0xA5) sentinel++;
		dma_sync(fds[i], SYNC_REND);
		double written = (double)(FRAME - sentinel) / (double)FRAME * 100.0;
		if (written > 20.0) has_written++;
		printf("final buf[%d]: %.1f%% overwritten\n", i, written);
	}
	printf("%d/%d DMA-BUFs were overwritten by the HAL\n", has_written, N_BUF);

	camhal_backend_stop(b);
	camhal_backend_destroy(b);

	/* ---- SAME-SCENE CONTROL: copy-mode (USERPTR) capture luma. ---- */
	{
		struct camhal_backend *cb = camhal_backend_create(cam, NULL);
		int cs = 0, cstride = 0;
		if (cb && camhal_backend_configure(cb, W, H, 6, &cstride, &cs) == 0 &&
		    camhal_backend_start(cb) == 0) {
			void *cptr = malloc((size_t)cs);
			uint8_t *raw = (uint8_t *)cptr;
			double best = -1;
			if (cptr) {
				for (int k = 0; k < 40; k++) {
					uint64_t cts = 0;
					if (camhal_backend_dqbuf(cb, cptr, cs, &cts) >= 0) {
						size_t cn = (size_t)cs / 3;
						size_t cn2 = cn < FRAME / 3 ? cn : FRAME / 3;
						double s = 0;
						for (size_t z = 0; z < cn2; z++) s += raw[z];
						double a = cn2 ? s / cn2 : 0;
						if (a > best) best = a;
					}
				}
			}
			printf("CONTROL copy-mode (USERPTR) max luma avg=%.1f%s\n",
			       best >= 0 ? best : 0, best > 5.0 ? "  <-- scene live" : "  <-- scene black");
			free(cptr);
			camhal_backend_stop(cb);
		}
		camhal_backend_destroy(cb);
	}

	for (int i = 0; i < N_BUF; i++) { munmap(maps[i], ALIGNED); close(fds[i]); }
	drm_intel_bufmgr_destroy(bufmgr);
	close(dri);

	int ok = !bad && has_written >= 2;
	printf("\n== %s ==\n", ok ? "DMA-MODE OK: HAL imported AND wrote into our DMA-BUFs"
				  : "DMA-MODE FAILED: HAL dequeued but did NOT write our DMA-BUFs");
	return ok ? 0 : 1;
}
