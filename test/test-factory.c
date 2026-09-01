/*
 * test-factory.c - verify the icamera SPA plugin loads and initializes.
 *
 * Loads build/libspa-icamera.so via the SPA plugin loader, enumerates the
 * factory, instantiates a handle and checks that the spa_node interface is
 * obtainable. This validates the plugin plumbing independent of a full
 * PipeWire/WirePlumber session.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <spa/utils/defs.h>
#include <spa/support/plugin.h>
#include <spa/support/plugin-loader.h>
#include <spa/support/log.h>
#include <spa/utils/dict.h>
#include <spa/node/node.h>
#include <spa/utils/type.h>

static void stdout_log(void *object, enum spa_log_level level,
		       const char *file, int line, const char *func,
		       const char *fmt, ...) SPA_PRINTF_FUNC(6, 7);

static void stdout_logv(void *object, enum spa_log_level level,
			const char *file, int line, const char *func,
			const char *fmt, va_list args) SPA_PRINTF_FUNC(6, 0);

static void stdout_log(void *object, enum spa_log_level level,
		       const char *file, int line, const char *func,
		       const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stdout, fmt, args);
	fprintf(stdout, "\n");
	va_end(args);
}

static void stdout_logv(void *object, enum spa_log_level level,
			const char *file, int line, const char *func,
			const char *fmt, va_list args)
{
	vfprintf(stdout, fmt, args);
	fprintf(stdout, "\n");
}

static void stdout_logt(void *object, enum spa_log_level level,
			const struct spa_log_topic *topic,
			const char *file, int line, const char *func,
			const char *fmt, ...) SPA_PRINTF_FUNC(7, 8);
static void stdout_logtv(void *object, enum spa_log_level level,
			 const struct spa_log_topic *topic,
			 const char *file, int line, const char *func,
			 const char *fmt, va_list args) SPA_PRINTF_FUNC(7, 0);

static void stdout_logt(void *object, enum spa_log_level level,
			const struct spa_log_topic *topic,
			const char *file, int line, const char *func,
			const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stdout, fmt, args);
	fprintf(stdout, "\n");
	va_end(args);
}

static void stdout_logtv(void *object, enum spa_log_level level,
			 const struct spa_log_topic *topic,
			 const char *file, int line, const char *func,
			 const char *fmt, va_list args)
{
	vfprintf(stdout, fmt, args);
	fprintf(stdout, "\n");
}

static const struct spa_log_methods log_methods = {
	SPA_VERSION_LOG_METHODS,
	.log = stdout_log,
	.logv = stdout_logv,
	.logt = stdout_logt,
	.logtv = stdout_logtv,
};

int main(int argc, char *argv[])
{
	struct spa_support support[1];
	struct spa_log log = {
		.iface = SPA_INTERFACE_INIT(SPA_TYPE_INTERFACE_Log,
					    SPA_VERSION_LOG, &log_methods, NULL),
	};
	const struct spa_handle_factory *factory = NULL;
	struct spa_handle *handle;
	void *iface = NULL;
	uint32_t index = 0;
	int res;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <path-to-libspa-icamera.so>\n", argv[0]);
		return 2;
	}

	support[0].type = SPA_TYPE_INTERFACE_Log;
	support[0].data = &log;

	res = spa_handle_factory_enum(&factory, &index);
	printf("factory_enum[0] = %d\n", res);
	if (res != 1 || factory == NULL) {
		fprintf(stderr, "no factory\n");
		return 1;
	}
	printf("factory name = %s\n", factory->name);

	res = spa_handle_factory_enum(&factory, &index);
	printf("factory_enum[1] = %d (expect 0)\n", res);

	factory = NULL;
	index = 0;
	spa_handle_factory_enum(&factory, &index);

	if (factory->get_size == NULL || factory->init == NULL ||
	    factory->enum_interface_info == NULL) {
		fprintf(stderr, "factory methods missing\n");
		return 1;
	}

	handle = calloc(1, factory->get_size(factory, NULL));
	if (handle == NULL) {
		fprintf(stderr, "calloc failed\n");
		return 1;
	}

	res = factory->init(factory, handle, NULL, support, 1);
	printf("init = %d (expect 0)\n", res);
	if (res < 0) {
		fprintf(stderr, "init failed\n");
		return 1;
	}

	res = handle->get_interface(handle, SPA_TYPE_INTERFACE_Node, &iface);
	printf("get_interface = %d (expect 0)\n", res);
	if (res < 0 || iface == NULL) {
		fprintf(stderr, "get_interface failed\n");
		return 1;
	}
	printf("node interface OK: %p\n", iface);

	handle->clear(handle);
	free(handle);
	printf("PASS\n");
	return 0;
}
