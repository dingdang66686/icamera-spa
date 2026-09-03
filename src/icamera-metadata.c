/* icamera-metadata.c - per-frame 3A metadata (SPA_META_Control) writer.
 *
 * The SPA node emits per-frame 3A results (AE exposure / ISO / frame rate /
 * AWB) to consumers that negotiated a SPA_META_Control metadata area.  That
 * bridge is a pure data -> pod transform, so it is kept as its own
 * translation unit and takes only the resolved metadata snapshot plus a log
 * handle -- no node/port/backend state.
 */
#include "icamera-metadata.h"

#include <string.h>

#include <spa/param/props.h>
#include <spa/control/control.h>
#include <spa/buffer/meta.h>
#include <spa/pod/builder.h>
#include <spa/pod/pod.h>

#include "camhal_backend.h"

/*
 * Per-frame 3A metadata (custom props keys inside the SPA_META_Control /
 * SPA_CONTROL_Properties sequence we advertise).  Custom key space starts at
 * SPA_PROP_START_Custom; offsets below are stable within this plugin.
 */
enum {
	ICAM_META_AE_STATE  = SPA_PROP_START_CUSTOM + 0, /* int  */
	ICAM_META_EXPOSURE  = SPA_PROP_START_CUSTOM + 1, /* long, us */
	ICAM_META_ISO       = SPA_PROP_START_CUSTOM + 2, /* int  */
	ICAM_META_FPS       = SPA_PROP_START_CUSTOM + 3, /* float */
	ICAM_META_AWB_R     = SPA_PROP_START_CUSTOM + 4, /* float r/g */
	ICAM_META_AWB_G     = SPA_PROP_START_CUSTOM + 5, /* float g/g */
	ICAM_META_AWB_B     = SPA_PROP_START_CUSTOM + 6, /* float b/g */
};

void icamera_metadata_fill(struct spa_meta_control *mc,
			   const struct camhal_metadata *m,
			   struct spa_log *log)
{
	struct spa_pod_builder b;
	uint8_t tmp[256];
	struct spa_pod_frame f_seq, f_obj;
	struct spa_pod *res;

	/* If the peer did not allocate a Control meta, or the backend does not
	 * have 3A values yet, there is nothing to publish. */
	if (mc == NULL || m == NULL)
		return;
	if (!m->valid)
		return;

	/* Build a sequence with a single SPA_CONTROL_Properties control whose
	 * value is a Props object carrying the 3A key/value pairs. */
	spa_pod_builder_init(&b, tmp, sizeof(tmp));
	spa_pod_builder_push_sequence(&b, &f_seq, 0);
	spa_pod_builder_control(&b, 0, SPA_CONTROL_Properties);
	spa_pod_builder_push_object(&b, &f_obj, SPA_TYPE_OBJECT_Props, SPA_PARAM_Props);
	spa_pod_builder_prop(&b, ICAM_META_AE_STATE, 0);
	spa_pod_builder_int(&b, m->ae_state);
	spa_pod_builder_prop(&b, ICAM_META_EXPOSURE, 0);
	spa_pod_builder_long(&b, m->exposure_us);
	spa_pod_builder_prop(&b, ICAM_META_ISO, 0);
	spa_pod_builder_int(&b, m->iso);
	spa_pod_builder_prop(&b, ICAM_META_FPS, 0);
	spa_pod_builder_float(&b, m->fps);
	spa_pod_builder_prop(&b, ICAM_META_AWB_R, 0);
	spa_pod_builder_float(&b, m->awb_r_per_g);
	spa_pod_builder_prop(&b, ICAM_META_AWB_G, 0);
	spa_pod_builder_float(&b, m->awb_g_per_g);
	spa_pod_builder_prop(&b, ICAM_META_AWB_B, 0);
	spa_pod_builder_float(&b, m->awb_b_per_g);
	spa_pod_builder_pop(&b, &f_obj);
	res = spa_pod_builder_pop(&b, &f_seq);
	if (res == NULL)
		return;

	/* Only blit when it fits the Control meta area reserved at negotiate
	 * time (meta.size accounts the whole spa_meta_control). */
	if (SPA_POD_SIZE(res) <= mc->sequence.pod.size) {
		memcpy(&mc->sequence, res, SPA_POD_SIZE(res));
		if (log)
			spa_log_debug(log,
				"icamera: 3A meta written ae=%d exp=%lldus iso=%d "
				"fps=%.1f awb_state=%d rgb=(%.2f,%.2f,%.2f)",
				m->ae_state, (long long)m->exposure_us, m->iso,
				m->fps, m->awb_state, m->awb_r_per_g,
				m->awb_g_per_g, m->awb_b_per_g);
	} else if (log)
		spa_log_debug(log, "icamera: 3A meta too large (%llu), skipped",
			      (unsigned long long)SPA_POD_SIZE(res));
}
