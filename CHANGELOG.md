# Changelog

All notable changes to `icamera-spa` are documented here.

This project adheres to [Semantic Versioning](https://semver.org/).
Until 1.0 the minor/patch fields are used loosely: 0.0.x marks a working
milestone, and the Z field increments whenever a tagged snapshot gains
substantive functionality on top of the previous one.

## [0.0.2] - 2026-09-17

First tagged snapshot with a complete, spec-compliant SPA source and
hardware zero-copy capture paths.  18 commits on top of `v0.0.1`
(4748 insertions / 307 deletions across 24 files).

### Added

- **Hardware zero-copy DMA-BUF capture (R-B)**: allocate i915 GEM DMA-BUFs and
  import them into the HAL via `V4L2_MEMORY_DMABUF`, so frames reach consumers
  without a CPU copy.  Gated behind a build macro (`dma-mode`).
- **HAL-supported pixel format enumeration**: the backend now exposes the
  formats/sizes the CamHAL actually advertises, and the plugin maps them
  through V4L2 fourcc → SPA video format; `EnumFormat`/`Format` negotiate
  against that list instead of a hardcoded set.
- **Stride passthrough**: HAL bytes-per-line is threaded through the capture
  path so non-4:3-aligned widths stop coming out sheared.
- **Frame-rate driven by 3A**: `EnumFormat`/`Format` fps now follows the
  configured 3A frame rate rather than a fixed 30/1.
- **Per-frame HAL 3A metadata**: exposure / gain / AE state are exported as
  `SPA_META_Control` on each buffer.
- **Node-level `PropInfo` for camera controls**: AE mode, AWB mode, AWB RGB
  gains, frame rate and 3A cadence are discoverable, so OBS's camera control
  panel and other generic consumers can read and set them.
- **Lazy libcamhal loading**: `libcamhal` is opened via `dlopen`/`dlsym` on
  demand (`camhal_loader.h`) instead of being linked at build time, so the
  plugin loads and errors cleanly on systems without it.
- **`media.type = Video` on icamera nodes**, so session managers classify them
  as video sources.
- **Tests**: `test-pw-reneg` (renegotiate regression), `test-pw-props` (SPA
  Props/PropInfo contract), `test-pw-dmabuf-direct`, `test-pw-dmabuf-consumer`,
  `test-hal-dmabuf`, `test-hal-release`, `test-hal-formats`.
- **`CHANGELOG.md`** (this file).

### Fixed

- **Use-after-free on runtime renegotiate**: `icamera_clear_buffers()` read the
  owned backing memory back through `frame->outbuf->datas[0]`, but during a
  renegotiate PipeWire may already have freed that `spa_buffer`.  The stale
  dereference crashed WirePlumber (and took the consuming app, e.g. OBS, with
  it) whenever the user picked a different resolution/framerate.  Buffer
  ownership is now tracked on the frame itself.
- **Camera not released on destroy**: a failed/duplicated teardown could leave
  the HAL device open, so a second open cycle failed.  Destroy now releases the
  camera exactly once.
- **Properties advertised in the wrong SPA namespace**: custom control ids were
  `0x10000..0x10008`, which is inside `SPA_PROP_START_Audio`.  They now live in
  `SPA_PROP_START_CUSTOM`, and `exposure`/`gain` use the standard
  `SPA_PROP_exposure` / `SPA_PROP_gain` ids so their registered types in
  `spa/param/props-types.h` apply.
- **`PropInfo` shape**: every entry is now wrapped in a Choice pod
  (`SPA_POD_CHOICE_RANGE_Int`/`_Float`, `SPA_POD_CHOICE_ENUM_Int`), fields are
  emitted in ascending id order, and `SPA_PROP_INFO_description` is always set
  (OBS uses it as the control label and ignores `SPA_PROP_INFO_name`).  The
  divergent port-only builder was deleted; node and port now serve one shared
  control table.
- **`SPA_PARAM_Props` completeness**: a value is now present for every
  advertised id, and `set_param` round-trips through the next enum.
- **Size-less / auto-negotiated `Format`** is accepted in
  `impl_node_port_set_param()` instead of being rejected.
- **Clock-aligned presentation timestamps** and a real-time capture thread.
- **Compiler warnings** in `icamera-source.c` eliminated.

### Changed

- **WirePlumber drop-in no longer disables the v4l2 and libcamera camera
  monitors.** That override was a debugging hack; it hid unrelated cameras from
  every application on the system.  `51-icamera.conf` now only registers the
  `api.icamera` factory, the three icamera monitor components and the
  `hardware.video-capture` aggregation, leaving v4l2/libcamera to load from the
  default profile.  Users who want to hide them can override at user level
  (`~/.config/wireplumber/wireplumber.conf.d/`).
- **Source split**: pure format and metadata helpers were extracted out of
  `icamera-source.c` into `src/icamera-format.[ch]` and
  `src/icamera-metadata.[ch]`.
- **Roadmap refreshed**: multi-format passthrough, zero-copy, clock and
  `PropInfo` status documented; pixel-format conversion explicitly scoped to
  downstream filters rather than the source.

## [0.0.1] - 2026-09-01

Initial tagged snapshot of the project (`8edf7fd`).  Baseline only: the
WirePlumber drop-in, the icamera monitor Lua scripts, and the first working
version of the SPA source node that pulls NV12 frames straight out of
`libcamhal`, bypassing GStreamer and v4l2loopback.

No functional changes are recorded for this release; it exists as the
starting point that 0.0.2 is measured against.

[0.0.2]: https://github.com/dingdang66686/icamera-spa/compare/v0.0.1...v0.0.2
[0.0.1]: https://github.com/dingdang66686/icamera-spa/releases/tag/v0.0.1
