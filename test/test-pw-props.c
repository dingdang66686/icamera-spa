/*
 * test-pw-props.c
 *
 * Regression test for the SPA Props / PropInfo contract of the icamera node.
 *
 * It pins down the things that used to be wrong (and that made OBS /
 * generic consumers unable to read the controls):
 *
 *   1. every advertised property id is in a legal namespace -- either a
 *      standard SPA video prop (exposure / gain) or >= SPA_PROP_START_CUSTOM.
 *      The old ids (0x10000..0x10008) collided with SPA_PROP_START_Audio.
 *   2. SPA_PROP_INFO_type is always a Choice pod (Range or Enum), which is
 *      what spa_pod_parse_object(..., SPA_PROP_INFO_type, SPA_POD_PodChoice())
 *      requires and what OBS / pw_stream parse.
 *   3. SPA_PROP_INFO_description is present (OBS uses it as the UI label and
 *      ignores SPA_PROP_INFO_name).
 *   4. Enum controls carry a matching SPA_PROP_INFO_labels struct.
 *   5. the node PropInfo channel and the port PropInfo channel advertise the
 *      same set of ids.
 *   6. SPA_PARAM_Props carries a value for every advertised id, and a
 *      set_param() round-trip is reflected back in the next Props enum.
 *
 * Build:
 *   gcc -O2 -g -Wall -o build/test-pw-props test/test-pw-props.c \
 *       $(pkg-config --cflags libpipewire-0.3)
 * Usage: ./build/test-pw-props [camera-name]
 * Exit 0 only if every check passes.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

#include <spa/support/log.h>
#include <spa/support/log-impl.h>
#include <spa/support/plugin.h>
#include <spa/utils/names.h>
#include <spa/utils/keys.h>
#include <spa/utils/result.h>
#include <spa/utils/string.h>
#include <spa/node/node.h>
#include <spa/monitor/device.h>
#include <spa/pod/builder.h>
#include <spa/pod/parser.h>
#include <spa/pod/vararg.h>
#include <spa/param/param.h>
#include <spa/param/props.h>
#include <spa/debug/pod.h>

#define SPA_PLUGIN_SUBDIR "icamera/libspa-icamera.so"
#define FACTORY_NAME "api.icamera.source"
#define MAX_PROPS 32

SPA_LOG_IMPL(default_log);

static const char *devname = "icamera_gc5035_rear";
static int failures;

#define CHECK(cond, ...) do {						\
	if (!(cond)) {							\
		failures++;						\
		fprintf(stderr, "FAIL: ");				\
		fprintf(stderr, __VA_ARGS__);				\
		fprintf(stderr, "\n");					\
	}								\
} while (0)

struct prop {
	uint32_t id;
	uint32_t choice;	/* SPA_CHOICE_* */
	uint32_t pod_type;	/* unwrapped value type */
	bool has_labels;
	bool has_description;
};

static int load_node(struct spa_node **node)
{
	char path[512];
	struct spa_dict_item items[2];
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
	items[0].key = SPA_KEY_DEVICE_NAME;
	items[0].value = devname;
	items[1].key = "node.name";
	items[1].value = "test-pw-props";
	struct spa_dict info = SPA_DICT_INIT_ARRAY(items);

	struct spa_support support[2];
	support[0] = SPA_SUPPORT_INIT(SPA_TYPE_INTERFACE_Log, &default_log.log);

	int res = spa_handle_factory_init(factory, handle, &info, support, 1);
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

/* Collect PropInfo from an enum_params() run into a listener. */
struct collect {
	struct prop props[MAX_PROPS];
	int n;
	uint32_t *ids;		/* also record ids for cross-channel compare */
	int *n_ids;
};

static void collect_result(void *data, int seq, int res, uint32_t type,
			   const void *result)
{
	struct collect *c = data;
	const struct spa_result_node_params *r = result;
	const struct spa_pod *pod_type, *labels = NULL;
	uint32_t id, choice = SPA_CHOICE_None, n_vals;
	int32_t ival;
	float fval;
	(void)seq; (void)res; (void)type;

	if (c->n >= MAX_PROPS)
		return;

	if (spa_pod_parse_object(r->param,
			SPA_TYPE_OBJECT_PropInfo, NULL,
			SPA_PROP_INFO_id, SPA_POD_Id(&id),
			SPA_PROP_INFO_type, SPA_POD_PodChoice(&pod_type),
			SPA_PROP_INFO_labels, SPA_POD_OPT_PodStruct(&labels)) < 0) {
		failures++;
		fprintf(stderr, "FAIL: PropInfo #%d does not parse\n", c->n);
		return;
	}

	struct prop *p = &c->props[c->n];
	p->id = id;
	p->has_labels = labels != NULL;

	/* description must be present and a String */
	const char *desc = NULL;
	if (spa_pod_parse_object(r->param,
			SPA_TYPE_OBJECT_PropInfo, NULL,
			SPA_PROP_INFO_description, SPA_POD_OPT_String(&desc)) < 0) {
		failures++;
		fprintf(stderr, "FAIL: id 0x%x has a non-String description\n", id);
	}
	p->has_description = desc != NULL;

	const struct spa_pod *v = spa_pod_get_values(pod_type, &n_vals, &choice);
	(void)ival; (void)fval;
	p->choice = choice;
	p->pod_type = n_vals > 0 ? SPA_POD_TYPE(v) : 0;

	if (c->n_ids && c->ids) {
		c->ids[*c->n_ids] = id;
		(*c->n_ids)++;
	}
	c->n++;
}

static int enum_propinfo(struct spa_node *node, bool port,
			 struct collect *c)
{
	struct spa_hook listener = { 0 };
	static const struct spa_node_events events = {
		SPA_VERSION_NODE_EVENTS,
		.result = collect_result,
	};
	int res;

	spa_node_add_listener(node, &listener, &events, c);
	if (port)
		res = spa_node_port_enum_params(node, 0, SPA_DIRECTION_OUTPUT, 0,
						SPA_PARAM_PropInfo, 0, MAX_PROPS, NULL);
	else
		res = spa_node_enum_params(node, 0, SPA_PARAM_PropInfo, 0, MAX_PROPS, NULL);
	spa_hook_remove(&listener);
	return res;
}

/* enum SPA_PARAM_Props and record which ids are present + their values. */
struct props_seen {
	uint32_t ids[MAX_PROPS];
	int n;
	int32_t ae_mode;
	bool have_ae;
};

static void props_result(void *data, int seq, int res, uint32_t type,
			 const void *result)
{
	struct props_seen *ps = data;
	const struct spa_result_node_params *r = result;
	struct spa_pod_object *obj = (struct spa_pod_object *)r->param;
	struct spa_pod_prop *prop;
	(void)seq; (void)res; (void)type;

	if (SPA_POD_TYPE(r->param) != SPA_TYPE_Object ||
	    SPA_POD_OBJECT_TYPE(r->param) != SPA_TYPE_OBJECT_Props)
		return;

	SPA_POD_OBJECT_FOREACH(obj, prop) {
		if (prop->key == SPA_PROP_params)
			continue;
		if (ps->n < MAX_PROPS)
			ps->ids[ps->n++] = prop->key;
		if (prop->key == SPA_PROP_START_CUSTOM + 0 &&
		    SPA_POD_TYPE(&prop->value) == SPA_TYPE_Int) {
			ps->ae_mode = SPA_POD_VALUE(struct spa_pod_int, &prop->value);
			ps->have_ae = true;
		}
	}
}

static int enum_props(struct spa_node *node, struct props_seen *ps)
{
	struct spa_hook listener = { 0 };
	static const struct spa_node_events events = {
		SPA_VERSION_NODE_EVENTS,
		.result = props_result,
	};
	int res;

	spa_node_add_listener(node, &listener, &events, ps);
	res = spa_node_enum_params(node, 0, SPA_PARAM_Props, 0, MAX_PROPS, NULL);
	spa_hook_remove(&listener);
	return res;
}

static bool props_seen_has(const struct props_seen *ps, uint32_t id)
{
	for (int i = 0; i < ps->n; i++)
		if (ps->ids[i] == id)
			return true;
	return false;
}

static bool is_legal_id(uint32_t id)
{
	return id == SPA_PROP_exposure || id == SPA_PROP_gain ||
	       id >= SPA_PROP_START_CUSTOM;
}

static int set_one_prop(struct spa_node *node, uint32_t id, int32_t value)
{
	uint8_t buf[512];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof(buf));
	const struct spa_pod *param;

	param = spa_pod_builder_add_object(&b,
		SPA_TYPE_OBJECT_Props, SPA_PARAM_Props,
		id, SPA_POD_Int(value));
	return spa_node_set_param(node, SPA_PARAM_Props, 0, param);
}

int main(int argc, char *argv[])
{
	struct spa_node *node = NULL;
	struct collect node_pi = { 0 }, port_pi = { 0 };
	uint32_t node_ids[MAX_PROPS], port_ids[MAX_PROPS];
	int n_node_ids = 0, n_port_ids = 0;
	int res;

	if (argc > 1)
		devname = argv[1];

	res = load_node(&node);
	if (res < 0)
		return 77;	/* skip: no camera available */

	node_pi.ids = node_ids;   node_pi.n_ids = &n_node_ids;
	port_pi.ids = port_ids;   port_pi.n_ids = &n_port_ids;

	res = enum_propinfo(node, false, &node_pi);
	CHECK(res >= 0, "node enum PropInfo returned %d %s", res, spa_strerror(res));
	CHECK(n_node_ids > 0, "node advertised no PropInfo");

	res = enum_propinfo(node, true, &port_pi);
	CHECK(res >= 0, "port enum PropInfo returned %d %s", res, spa_strerror(res));

	printf("node PropInfo: %d, port PropInfo: %d\n", n_node_ids, n_port_ids);

	for (int i = 0; i < node_pi.n; i++) {
		struct prop *p = &node_pi.props[i];

		CHECK(is_legal_id(p->id),
		      "id 0x%x is outside the legal namespace "
		      "(expected SPA_PROP_exposure/gain or >= 0x%x)",
		      p->id, SPA_PROP_START_CUSTOM);
		CHECK(p->choice == SPA_CHOICE_Range || p->choice == SPA_CHOICE_Enum,
		      "id 0x%x type is not a Range/Enum Choice (choice=%u)",
		      p->id, p->choice);
		CHECK(p->pod_type == SPA_TYPE_Int || p->pod_type == SPA_TYPE_Float ||
		      p->pod_type == SPA_TYPE_Bool,
		      "id 0x%x has unsupported value type %u",
		      p->id, p->pod_type);
		CHECK(p->has_description,
		      "id 0x%x has no SPA_PROP_INFO_description", p->id);
		if (p->choice == SPA_CHOICE_Enum)
			CHECK(p->has_labels,
			      "id 0x%x is an Enum choice without labels", p->id);
	}

	/* node and port must advertise the same id set */
	CHECK(n_port_ids == n_node_ids,
	      "port advertises %d props but node advertises %d",
	      n_port_ids, n_node_ids);
	for (int i = 0; i < n_node_ids; i++) {
		bool found = false;
		for (int j = 0; j < n_port_ids; j++)
			if (port_ids[j] == node_ids[i])
				found = true;
		CHECK(found, "id 0x%x missing from the port PropInfo", node_ids[i]);
	}

	/* every advertised id must have a value in SPA_PARAM_Props */
	{
		struct props_seen ps = { 0 };
		res = enum_props(node, &ps);
		CHECK(res >= 0, "enum Props returned %d %s", res, spa_strerror(res));
		for (int i = 0; i < n_node_ids; i++)
			CHECK(props_seen_has(&ps, node_ids[i]),
			      "id 0x%x advertised in PropInfo but absent from Props",
			      node_ids[i]);
	}

	/* set_param round-trip: write a new AE mode, expect it back */
	{
		struct props_seen ps = { 0 };
		int32_t want = 1;

		res = set_one_prop(node, SPA_PROP_START_CUSTOM + 0 /* ae-mode */, want);
		CHECK(res >= 0, "set_param(ae-mode) returned %d %s",
		      res, spa_strerror(res));

		res = enum_props(node, &ps);
		CHECK(res >= 0, "post-set enum Props returned %d %s",
		      res, spa_strerror(res));
		CHECK(ps.have_ae, "ae-mode missing from Props after set_param");
		CHECK(ps.ae_mode == want,
		      "ae-mode round-trip: wrote %d, read back %d",
		      want, ps.ae_mode);
	}

	if (failures == 0)
		printf("PASS: SPA Props/PropInfo contract is spec compliant\n");
	else
		fprintf(stderr, "%d check(s) failed\n", failures);
	return failures == 0 ? 0 : 1;
}
