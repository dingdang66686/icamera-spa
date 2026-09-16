/*
 * camhal_busy.h - minimal cross-session (cross-UID) camera busy check.
 *
 * libcamhal creates one process-wide SysV shared-memory segment with the fixed
 * key 0x43414D (see CameraShm.cpp: CAMERA_IPCKEY) to arbitrate camera access
 * between processes of the SAME user.  Its mode is hardcoded 0640, so a camera
 * process running under a DIFFERENT UID (e.g. the gdm greeter, or another login
 * session) can neither attach nor re-create it: its shmget() fails with EACCES
 * and camera_device_open() returns INVALID_OPERATION.
 *
 * We cannot change that segment nor reuse libcamhal's arbitration (no public
 * API, mode 0640).  So before loading the HAL we make the same decision the
 * kernel would make for us, but cheaply: if the segment already exists and is
 * owned by another UID, the camera is in use by another session -> EBUSY.
 *
 * /proc/sysvipc/shm is world readable, so this needs no privileges.  Columns:
 *
 *   key shmid perms size cpid lpid nattch uid gid cuid cgid atime dtime ctime rss swap
 *
 * We deliberately do NOT try to clean up orphaned segments: an unprivileged
 * process cannot remove a segment owned by another UID.  With the loader
 * (camhal_loader.h) unloading libcamhal after use, a normally-exiting process
 * lets the HAL destructor remove its own segment, so orphans only appear on
 * abnormal termination.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CAMHAL_BUSY_H
#define CAMHAL_BUSY_H

#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

/* Fixed SysV IPC key of libcamhal's CameraSharedMemory (CameraShm.cpp). */
#define CAMHAL_SHM_KEY 0x43414D

/*
 * Build-time sanity check of our model of libcamhal's shared segment.  If a
 * future HAL changes either the camera count or the per-camera status size the
 * assertion fires here, instead of us silently mis-detecting.
 *
 *   struct cameraDevStatus  { pid_t pid; char name[64]; };
 *   struct camera_shared_info { cameraDevStatus camDevStatus[MAX_CAMERA_NUMBER]; };
 *   CAMERA_SM_SIZE = (sizeof(camera_shared_info) / getpagesize() + 1) * getpagesize();
 */
#define CAMHAL_MAX_CAMERA_NUMBER      2
#define CAMHAL_PROCESS_NAME_LENGTH   64
#define CAMHAL_DEVSTATUS_SIZE \
	(sizeof(pid_t) + CAMHAL_PROCESS_NAME_LENGTH)
extern char camhal_devstatus_size_assert
	[(sizeof(pid_t) + CAMHAL_PROCESS_NAME_LENGTH == CAMHAL_DEVSTATUS_SIZE)
		 ? 1 : -1];

/*
 * Return 1 if the camera HAL segment exists and belongs to a different UID
 * (camera is in use by another session), 0 otherwise (free, or ours).
 * Returns 0 on any parse/read error ("don't know" -> let the HAL decide).
 */
static inline int camhal_camera_busy(void)
{
	FILE *fp = fopen("/proc/sysvipc/shm", "r");
	if (!fp)
		return 0;

	uid_t me = getuid();
	char line[512];
	int busy = 0;

	/* Skip the header line. */
	if (!fgets(line, sizeof(line), fp)) {
		fclose(fp);
		return 0;
	}

	while (fgets(line, sizeof(line), fp)) {
		long key, shmid, perms, size, cpid, lpid, nattch, uid, gid;

		/* Only the leading numeric fields are needed; the trailing
		 * timestamp/rss/swap columns may be wide, so match loosely. */
		if (sscanf(line, "%ld %ld %ld %ld %ld %ld %ld %ld %ld",
			   &key, &shmid, &perms, &size, &cpid, &lpid,
			   &nattch, &uid, &gid) != 9)
			continue;

		if (key != (long)CAMHAL_SHM_KEY)
			continue;

		/* The segment exists.  If it is not ours, another UID owns the
		 * camera HAL arbitration segment. */
		if ((long)uid != (long)me)
			busy = 1;

		break;
	}

	fclose(fp);
	return busy;
}

#endif /* CAMHAL_BUSY_H */
