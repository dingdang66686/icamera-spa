/* icamera-metadata.h - per-frame 3A metadata (SPA_META_Control) writer */
#ifndef ICAMERA_METADATA_H
#define ICAMERA_METADATA_H

#include <spa/support/log.h>

/* Forward decl: full definition lives in spa/buffer/buffer.h */
struct spa_meta_control;
/* Forward decl: full definition lives in camhal_backend.h */
struct camhal_metadata;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fill the SPA_META_Control area of a frame with per-frame 3A results as a
 * SPA_CONTROL_Properties sequence, using the given backend metadata snapshot.
 *
 * This is a pure data -> pod transform: it carries no node/port/backend
 * state of its own.  If mc is NULL (peer did not allocate a Control meta) or
 * the metadata snapshot is not yet valid, nothing is written.
 */
void icamera_metadata_fill(struct spa_meta_control *mc,
			   const struct camhal_metadata *m,
			   struct spa_log *log);

#ifdef __cplusplus
}
#endif

#endif /* ICAMERA_METADATA_H */
