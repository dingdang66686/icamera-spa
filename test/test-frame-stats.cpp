// test-frame-stats.cpp - verify USERPTR frames carry real image data.
// Build:
//   g++ -O2 -Wall -Isrc -o build/test-frame-stats test/test-frame-stats.cpp ./build/libspa-icamera.so
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "camhal_backend.h"

static void analyze(const uint8_t *p, long n, const char *tag)
{
        unsigned long long sum = 0; unsigned long long sum2 = 0;
        int mn = 255, mx = 0; long nz = 0, n255 = 0;
        for (long i = 0; i < n; i++) {
                int v = p[i];
                sum += v; sum2 += (unsigned)v * v;
                if (v < mn) mn = v;
                if (v > mx) mx = v;
                if (v) nz++;
                if (v == 255) n255++;
        }
        double mean = (double)sum / n;
        double var = (double)sum2 / n - mean * mean;
        printf("%s: size=%ld min=%d max=%d mean=%.2f stddev=%.2f "
               "nonzero=%ld (%.1f%%) all255=%ld\n",
               tag, n, mn, mx, mean, var > 0 ? __builtin_sqrt(var) : 0.0,
               nz, 100.0 * nz / n, n255);
}

int main(void)
{
        int count = 0;
        struct camhal_camera_info *cams = camhal_discover_cameras(&count);
        if (!cams) { printf("discover failed\n"); return 1; }
        printf("found %d cameras\n", count);

        for (int i = 0; i < count; i++) {
                int id = cams[i].camera_id;
                printf("=== cam %s (id=%d) ===\n", cams[i].name, id);
                struct camhal_backend *b = camhal_backend_create(id, NULL);
                if (!b) { printf("  create failed\n"); continue; }
                int stride = 0, size = 0;
                if (camhal_backend_configure(b, 0x3231564e /* NV12 */,
                                             640, 480, 4, &stride, &size) < 0) {
                        printf("  configure failed\n"); camhal_backend_destroy(b); continue;
                }
                printf("  configure OK stride=%d size=%d\n", stride, size);
                if (camhal_backend_start(b) < 0) {
                        printf("  start failed\n"); camhal_backend_destroy(b); continue;
                }
                /* capture 2 frames */
                for (int f = 0; f < 2; f++) {
                        uint8_t *frame = (uint8_t *)malloc(size);
                        uint64_t ts = 0;
                        if (camhal_backend_dqbuf(b, frame, size, &ts) < 0) {
                                printf("  dqbuf failed\n"); free(frame); break;
                        }
                        char tag[64];
                        snprintf(tag, sizeof(tag), "cam %d frame %d ts=%llu",
                                 id, f, (unsigned long long)ts);
                        analyze(frame, size, tag);
                        free(frame);
                }
                camhal_backend_stop(b);
                camhal_backend_destroy(b);
        }
        camhal_free_cameras(cams, count);
        return 0;
}
