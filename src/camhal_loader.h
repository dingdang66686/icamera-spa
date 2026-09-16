/*
 * camhal_loader.h - on-demand loader for the Intel libcamhal shared library.
 *
 * Why: libcamhal runs a global constructor (__attribute__((constructor)))
 * that instantiates CameraHal and, as a side effect, creates a process-wide
 * SysV shared-memory segment (fixed key 0x43414D, mode 0640) used for
 * cross-process camera arbitration.  Linking libcamhal.so.0 directly (DT_NEEDED)
 * therefore means "load the camera HAL and create that segment" the moment the
 * plugin is loaded into ANY process - even one that only wants to enumerate
 * cameras.  That breaks down with two concurrent desktop sessions of different
 * UIDs: the first session's 0640 segment blocks the second (EACCES on shmget).
 *
 * Instead we dlopen() libcamhal.so.0 only when the HAL is actually needed and
 * unload it as soon as the last user is gone: "use it, then remove every
 * trace".
 *
 * Subtlety - the SysV segment is owned by the HAL *plugin*, not the main .so.
 * The main libcamhal.so.0 is only a thin adaptor that dlopen()s a per-platform
 * plugin (e.g. libcamhal/plugins/ipu6.so) which carries CameraShm and the
 * __attribute__((constructor)) that builds the CameraHal instance (and the
 * segment).  That plugin exports GNU_UNIQUE symbols (libstdc++
 * make_shared __tag), so glibc marks it RTLD_NODELETE: dlclose() never unmaps
 * it and its destructor never runs, leaving the segment behind forever.
 *
 * We therefore drive the plugin's own initCameraHAL()/deinitCameraHAL() entry
 * points by hand: on the first dlopen the plugin constructor already built the
 * instance; on every later load we recreate it, and on the last release we call
 * deinitCameraHAL() to delete the instance - which detaches and IPC_RMID()s the
 * segment - before unmapping the (freely unloadable) main library.
 *
 * The loader is reference counted and thread safe.  Discovery call sites that
 * do not otherwise hold a backend use the RAII `guard`, so the segment exists
 * only for the duration of the enumeration.
 *
 * Escape hatch: set ICAMERA_KEEP_HAL=1 to open the library with RTLD_NODELETE
 * and keep the HAL instance (and its segment) alive across releases - useful if
 * a HAL build has an unsafe static destructor.  Set
 * ICAMERA_LIBCAMHAL=<path> to load a different SONAME.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CAMHAL_LOADER_H
#define CAMHAL_LOADER_H

#include <dlfcn.h>
#include <errno.h>
#include <link.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Types of the HAL ABI.  These come from the SDK headers and are only needed at
 * compile time; the symbols themselves are resolved with dlsym() at run time. */
#include <libcamhal/api/ICamera.h>
#include <libcamhal/api/Parameters.h>

namespace icamera_loader {

using icamera::camera_buffer_t;
using icamera::camera_info_t;
using icamera::Parameters;
using icamera::stream_config_t;
using icamera::stream_t;

/* Resolved exports of libcamhal.so.0 (all plain C linkage). */
struct table {
	int  (*get_number_of_cameras)(void);
	int  (*get_camera_info)(int, camera_info_t &);
	int  (*get_frame_size)(int, int, int, int, int, int *);
	int  (*camera_hal_init)(void);
	int  (*camera_hal_deinit)(void);
	int  (*camera_device_open)(int, int);
	void (*camera_device_close)(int);
	int  (*camera_device_config_sensor_input)(int, const stream_t *);
	int  (*camera_device_config_streams)(int, stream_config_t *);
	int  (*camera_device_start)(int);
	int  (*camera_device_stop)(int);
	int  (*camera_device_allocate_memory)(int, camera_buffer_t *);
	int  (*camera_stream_qbuf)(int, camera_buffer_t **, int, const Parameters *);
	int  (*camera_stream_dqbuf)(int, int, camera_buffer_t **, Parameters *);
	int  (*camera_set_parameters)(int, const Parameters &);
};

/*
 * icamera::Parameters is a C++ class with NO virtual functions and a single
 * `void *mData` member (sizeof == pointer size), so it can be constructed and
 * driven entirely through dlsym()'d member-function pointers.  C++ member
 * functions take `this` as their first argument (Itanium C++ ABI), so each is
 * modelled here as a free function taking the object pointer first.
 *
 * Why bother: the backend builds a Parameters to push 3A settings and reads one
 * back from dqbuf for metadata.  Leaving those calls as direct member calls
 * would leave 17 undefined `_ZN7icamera10Parameters*` symbols in the plugin,
 * forcing a DT_NEEDED on libcamhal.so.0 - i.e. loading the HAL (and creating
 * its shared-memory segment) the moment the SPA plugin is dlopen()ed, which is
 * exactly what we are trying to avoid.  Routing them through dlsym() keeps the
 * plugin free of any libcamhal dependency.
 *
 * The mangled names below are the GCC/Itanium encodings of this HAL's ABI; they
 * are stable for a given libcamhal soname.
 */
struct par {
	void (*c_void)(void *);					/* Parameters::Parameters()   */
	void (*d_void)(void *);					/* Parameters::~Parameters()  */
	int  (*set_AeMode)(void *, icamera::camera_ae_mode_t);
	int  (*set_ExposureTime)(void *, int64_t);
	int  (*set_SensitivityGain)(void *, float);
	int  (*set_AwbMode)(void *, icamera::camera_awb_mode_t);
	int  (*set_AwbGains)(void *, icamera::camera_awb_gains_t);
	int  (*set_FrameRate)(void *, float);
	int  (*set_Run3ACadence)(void *, int);
	int  (*get_AeState)(const void *, icamera::camera_ae_state_t *);
	int  (*get_ExposureTime)(const void *, int64_t *);
	int  (*get_SensitivityIso)(const void *, int *);
	int  (*get_FrameRate)(const void *, float *);
	int  (*get_AwbState)(const void *, icamera::camera_awb_state_t *);
	int  (*get_AwbResult)(const void *, void *);
	int  (*get_AwbGains)(const void *, icamera::camera_awb_gains_t *);
	int  (*get_SupportedStreamConfig)(const void *, std::vector<icamera::stream_t> *);
};

struct state {
	void *handle;
	int   refs;
	int   permanent;	/* ICAMERA_KEEP_HAL: keep the table, never unload */
	int   ctor_done;	/* plugin ctor already built the first instance */
	int   live;		/* an instance is currently built                */
	void *plugin;		/* NOLOAD handle to the per-platform plugin    */
	void (*plugin_init)(void);	/* exported initCameraHAL()            */
	void (*plugin_deinit)(void);	/* exported deinitCameraHAL()          */
	struct table f;
	struct par p;
	pthread_mutex_t lock;
};

inline struct state &st(void)
{
	static struct state s = { NULL, 0, 0, 0, 0, NULL, NULL, NULL,
				  {}, {}, PTHREAD_MUTEX_INITIALIZER };
	return s;
}

/* The table of resolved symbols.  Only valid while a reference is held. */
inline struct table &fn(void)
{
	return st().f;
}

/* Resolved icamera::Parameters member functions (same lifetime as fn()). */
inline struct par &par(void)
{
	return st().p;
}

/*
 * Stack storage for an icamera::Parameters built via the dlsym()'d ctor/dtor.
 * `get()` yields the object address to pass as `this`; `ref()` yields a real
 * icamera::Parameters reference for the few APIs that take one by reference.
 */
struct parameters {
	alignas(Parameters) unsigned char obj[sizeof(Parameters)];
	parameters(void) { par().c_void(obj); }
	~parameters(void) { par().d_void(obj); }
	void *get(void) const { return (void *)obj; }
	const Parameters &ref(void) const { return *(const Parameters *)obj; }
	parameters(const parameters &) = delete;
	parameters &operator=(const parameters &) = delete;
};

inline const char *libname(void)
{
	const char *e = getenv("ICAMERA_LIBCAMHAL");
	return (e && *e) ? e : "libcamhal.so.0";
}

/* Do NOT pin the library (allow unload to really drop the main library). */
inline bool unload_enabled(void)
{
	const char *e = getenv("ICAMERA_KEEP_HAL");
	return !(e && e[0] == '1');
}

/* dl_iterate_phdr callback: find the loaded per-platform HAL plugin. */
inline int find_plugin_cb(struct dl_phdr_info *info, size_t size, void *data)
{
	(void)size;
	const char *name = info->dlpi_name;
	if (name && *name &&
	    strstr(name, "libcamhal/plugins/") && strstr(name, ".so")) {
		snprintf((char *)data, 4095, "%s", name);
		return 1;	/* stop */
	}
	return 0;
}

/* Locate the plugin the main adaptor dlopen()ed (it carries CameraShm and the
 * initCameraHAL/deinitCameraHAL constructors, and is RTLD_NODELETE). */
inline void bind_plugin_locked(struct state &s)
{
	if (s.plugin)
		return;

	char path[4096] = { 0 };
	dl_iterate_phdr(find_plugin_cb, path);
	if (!path[0])
		return;

	/* The plugin is already mapped; RTLD_NOLOAD just hands us a handle. */
	void *p = dlopen(path, RTLD_NOW | RTLD_LOCAL | RTLD_NOLOAD);
	if (!p)
		return;

	s.plugin = p;
	s.plugin_init = (void (*)(void))dlsym(p, "initCameraHAL");
	s.plugin_deinit = (void (*)(void))dlsym(p, "deinitCameraHAL");
}

/* Build a HAL instance (and its SysV segment) inside the NODELETE plugin.  The
 * very first dlopen of the main library already ran the plugin constructor,
 * which does exactly this, so we skip the first time and only rebuild for
 * subsequent acquire()s. */
inline void plugin_init_locked(struct state &s)
{
	if (s.live)
		return;
	if (!s.ctor_done) {
		/* The plugin constructor already created the first instance. */
		s.ctor_done = 1;
	} else if (s.plugin_init) {
		s.plugin_init();
	}
	s.live = 1;
}

/* Delete the HAL instance (segments included) inside the plugin.  This is what
 * a working dlclose() would have triggered via __attribute__((destructor)). */
inline void plugin_deinit_locked(struct state &s)
{
	if (s.permanent)
		return;		/* ICAMERA_KEEP_HAL: keep the instance alive */
	if (!s.live)
		return;
	if (s.plugin_deinit)
		s.plugin_deinit();
	s.live = 0;
}

/* Resolve every symbol.  Caller holds st().lock and refs == 0. */
inline int bind_locked(struct state &s)
{
	int flags = RTLD_NOW | RTLD_LOCAL;
	if (!unload_enabled())
		flags |= RTLD_NODELETE;

	void *h = dlopen(libname(), flags);
	if (!h) {
		fprintf(stderr, "icamera: dlopen(%s) failed: %s\n",
			libname(), dlerror());
		return -EIO;
	}

#define CAMHAL_BIND(lhs, name)                                          \
	do {                                                            \
		*(void **)(&(lhs)) = dlsym(h, name);                    \
		if (!(lhs)) {                                           \
			fprintf(stderr, "icamera: dlsym(%s) failed: %s\n", \
				name, dlerror());                       \
			dlclose(h);                                     \
			return -EIO;                                    \
		}                                                       \
	} while (0)

	CAMHAL_BIND(s.f.get_number_of_cameras, "get_number_of_cameras");
	CAMHAL_BIND(s.f.get_camera_info, "get_camera_info");
	CAMHAL_BIND(s.f.get_frame_size, "get_frame_size");
	CAMHAL_BIND(s.f.camera_hal_init, "camera_hal_init");
	CAMHAL_BIND(s.f.camera_hal_deinit, "camera_hal_deinit");
	CAMHAL_BIND(s.f.camera_device_open, "camera_device_open");
	CAMHAL_BIND(s.f.camera_device_close, "camera_device_close");
	CAMHAL_BIND(s.f.camera_device_config_sensor_input,
		    "camera_device_config_sensor_input");
	CAMHAL_BIND(s.f.camera_device_config_streams,
		    "camera_device_config_streams");
	CAMHAL_BIND(s.f.camera_device_start, "camera_device_start");
	CAMHAL_BIND(s.f.camera_device_stop, "camera_device_stop");
	CAMHAL_BIND(s.f.camera_device_allocate_memory,
		    "camera_device_allocate_memory");
	CAMHAL_BIND(s.f.camera_stream_qbuf, "camera_stream_qbuf");
	CAMHAL_BIND(s.f.camera_stream_dqbuf, "camera_stream_dqbuf");
	CAMHAL_BIND(s.f.camera_set_parameters, "camera_set_parameters");

	/* icamera::Parameters member functions (see struct par above). */
	CAMHAL_BIND(s.p.c_void, "_ZN7icamera10ParametersC1Ev");
	CAMHAL_BIND(s.p.d_void, "_ZN7icamera10ParametersD1Ev");
	CAMHAL_BIND(s.p.set_AeMode,
		    "_ZN7icamera10Parameters9setAeModeENS_16camera_ae_mode_tE");
	CAMHAL_BIND(s.p.set_ExposureTime,
		    "_ZN7icamera10Parameters15setExposureTimeEl");
	CAMHAL_BIND(s.p.set_SensitivityGain,
		    "_ZN7icamera10Parameters18setSensitivityGainEf");
	CAMHAL_BIND(s.p.set_AwbMode,
		    "_ZN7icamera10Parameters10setAwbModeENS_17camera_awb_mode_tE");
	CAMHAL_BIND(s.p.set_AwbGains,
		    "_ZN7icamera10Parameters11setAwbGainsENS_18camera_awb_gains_tE");
	CAMHAL_BIND(s.p.set_FrameRate, "_ZN7icamera10Parameters12setFrameRateEf");
	CAMHAL_BIND(s.p.set_Run3ACadence,
		    "_ZN7icamera10Parameters15setRun3ACadenceEi");
	CAMHAL_BIND(s.p.get_AeState,
		    "_ZNK7icamera10Parameters10getAeStateERNS_17camera_ae_state_tE");
	CAMHAL_BIND(s.p.get_ExposureTime,
		    "_ZNK7icamera10Parameters15getExposureTimeERl");
	CAMHAL_BIND(s.p.get_SensitivityIso,
		    "_ZNK7icamera10Parameters17getSensitivityIsoERi");
	CAMHAL_BIND(s.p.get_FrameRate,
		    "_ZNK7icamera10Parameters12getFrameRateERf");
	CAMHAL_BIND(s.p.get_AwbState,
		    "_ZNK7icamera10Parameters11getAwbStateERNS_18camera_awb_state_tE");
	CAMHAL_BIND(s.p.get_AwbResult,
		    "_ZNK7icamera10Parameters12getAwbResultEPv");
	CAMHAL_BIND(s.p.get_AwbGains,
		    "_ZNK7icamera10Parameters11getAwbGainsERNS_18camera_awb_gains_tE");
	CAMHAL_BIND(s.p.get_SupportedStreamConfig,
		    "_ZNK7icamera10Parameters24getSupportedStreamConfigERSt6vectorINS_8stream_tESaIS2_EE");

#undef CAMHAL_BIND

	s.handle = h;
	s.permanent = !unload_enabled();
	return 0;
}

/* Take a reference; dlopen()+resolve+build on the 0 -> 1 transition. */
inline int acquire(void)
{
	struct state &s = st();
	pthread_mutex_lock(&s.lock);
	if (s.refs > 0) {
		s.refs++;
		pthread_mutex_unlock(&s.lock);
		return 0;
	}
	int ret = bind_locked(s);
	if (ret < 0) {
		pthread_mutex_unlock(&s.lock);
		return ret;
	}
	bind_plugin_locked(s);
	plugin_init_locked(s);
	s.refs = 1;
	pthread_mutex_unlock(&s.lock);
	return 0;
}

/* Drop a reference; on the 1 -> 0 transition delete the HAL instance (which
 * IPC_RMID()s the SysV segment) and unmount the main library.  The plugin
 * itself is RTLD_NODELETE and stays mapped, so we keep its handle around. */
inline void release(void)
{
	struct state &s = st();
	pthread_mutex_lock(&s.lock);
	if (s.refs > 0) {
		s.refs--;
		if (s.refs == 0) {
			plugin_deinit_locked(s);
			if (s.handle && !s.permanent) {
				dlclose(s.handle);
				s.handle = NULL;
			}
			memset(&s.f, 0, sizeof(s.f));
			memset(&s.p, 0, sizeof(s.p));
		}
	}
	pthread_mutex_unlock(&s.lock);
}

/* RAII guard for call sites that do not hold a backend (discovery / format
 * query).  Holding the guard keeps fn() valid for the enclosing scope. */
struct guard {
	bool held;
	guard(void) : held(acquire() == 0) {}
	~guard(void) { if (held) release(); }
	bool ok(void) const { return held; }
};

}  /* namespace icamera_loader */

#endif /* CAMHAL_LOADER_H */
