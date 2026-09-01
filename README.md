# icamera-spa

一个基于 **Intel IPU6 Camera HAL (libcamhal)** 的原生 PipeWire SPA source 插件。

它直接从 `libcamhal` 拉取 NV12 帧，**完全绕过 GStreamer 和 v4l2loopback**，把相机以
PipeWire `Video/Source` 节点呈现给桌面/应用（如 Camera、Video 会议、portal 等）。

> 项目目标是替代传统 `icamerasrc`(GStreamer) 相机采集路径, 让相机节点直接进入
> PipeWire 图, 并支持**运行时动态修改 3A 参数**（曝光/增益/AWB）而不必重启节点。

> **开发方式声明**: 本项目采用 **Vibe Coding（AI 辅助生成）+ 人工审核、验证**的方式
> 完成。核心代码由 AI 辅助编写生成, 经开发者逐段人工审查、交叉验证（构建编译、
> 实机抓帧、动态 3A 闭环、日志/管线分析等）后确认。凡是会覆盖系统组件、重启服务或
> 影响系统行为的改动, 均经过人工审批后才落地。

---

## 特性

- **原生 SPA source 节点**: 完整实现 `spa_node` 数据面（生产线程 + `process()` + buffer 复用）。
- **直接对接 libcamhal**: C/C++ 桥接 C 接口, 说话 `icamera::` C++ API（`ICamera.h`）。
- **多 buffer 在途管线**: 复刻 `icamerasrc` 的 qbuf/dqbuf 节奏（`INFLIGHT_DEPTH=6`），
  避免 HAL 请求/3A/PSYS 管线因单 buffer 而 stall。
- **动态分辨率枚举**: 通过 `getSupportedStreamConfig()` 查询 HAL 真实支持的 NV12 分辨率，
  无需打开摄像头即可在节点 init 时枚举。
- **运行时动态 3A 控制**: 通过 SPA `Props` 的 `params` 通道, 用 `pw-cli` 即可在 streaming
  中修改曝光/增益/AWB, 无需重启节点。
- **WirePlumber 自动发现**: Lua monitor + 原生 Lua C 模块动态枚举真实内置相机
  （排除 USB/UVC）, 自动创建 `Video/Source` 节点, 正确区分前/后摄镜像。

---

## 架构

```
┌──────────────────────────── icamera-spa ───────────────────────────┐
│                                                                    │
│  PipeWire / WirePlumber                                             │
│    │  spa-node-factory("api.icamera.source")                       │
│    ▼                                                               │
│  ┌─────────────────────────────┐          ┌─────────────────────┐  │
│  │  icamera-source.c (SPA 节点) │          │ WirePlumber monitor │  │
│  │  - 数据面 / 格式 / buffer     │          │  - enumerate-device│  │
│  │  - 3A Props 动态通道 set/get │          │  - create-node      │  │
│  │  - 生产线程 (qbuf/dqbuf)     │          │  - name-node        │  │
│  └──────────────┬──────────────┘          └──────────┬──────────┘  │
│                 │ C 接口                               │ C (Lua C)   │
│                 ▼                                     ▼             │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │  camhal_backend.cpp  (C++ ⇄ libcamhal icamera:: API)          │ │
│  │  - camera_hal_init / camera_device_open / config_streams      │ │
│  │  - 多 buffer qbuf/dqbuf 在途管线                               │ │
│  │  - camera_set_parameters (3A)                                  │ │
│  └───────────────────────────────┬───────────────────────────────┘ │
│                                  ▼                                 │
│                     Intel IPU6 libcamhal (icamera)                 │
└────────────────────────────────────────────────────────────────────┘
```

**文件划分**

| 文件 | 语言 | 职责 |
|------|------|------|
| `src/icamera-source.c` | C | SPA 节点: 数据面、格式协商、buffer 分配、动态 3A Props 通道、工厂 |
| `src/camhal_backend.cpp` | C++ | libcamhal 采集后端桥接（因为 libcamhal 是 C++ API）|
| `src/camhal_backend.h` | C | 后端 C 接口（发现/配置/采集/3A）|
| `src/camhal-list.cpp` | C++ | 独立相机发现辅助程序 |
| `src/camhal_lua.cpp` | C++ | 给 WirePlumber Lua 沙箱用的原生发现模块 |
| `monitors/icamera/*.lua` | Lua | WirePlumber 监视器脚本 |
| `wireplumber/51-icamera.conf` | WP cfg | WirePlumber drop-in 配置 |

---

## 依赖

- **Intel IPU6 Camera HAL (libcamhal / `libcamhal.so`)** 及其头文件
  （`libcamhal/api/ICamera.h`, `libcamhal/api/Parameters.h`）
- **PipeWire / SPA 头文件**
  - SPA 头: `/usr/include/spa-0.2/`
  - libpipewire 头: `/usr/include/pipewire-0.3/`
- **WirePlumber**（用于自动 monitor 集成）
- 编译器: `gcc` + `g++`

本项目的开发/测试环境: **CachyOS (Arch-based)**, PipeWire 1.6.8, WirePlumber, Linux
Intel IPU6 平台。

---

## 构建

```sh
make all
```

产物:
- `build/libspa-icamera.so` — SPA 插件
- `build/camhal-list` — 相机发现命令行工具
- `build/camhal.so` — WirePlumber 用 Lua C 模块

---

## 安装

### 1) 安装 SPA 插件

```sh
sudo make install
```

安装到 `/usr/lib/spa-0.2/icamera/libspa-icamera.so`。

### 2) 安装 WirePlumber 集成（monitor + Lua 模块 + drop-in 配置）

```sh
sudo make install-monitor
```

这会安装:
- Lua 模块 → `/usr/lib/lua/<ver>/camhal.so`
- monitor 脚本 → `/usr/share/wireplumber/scripts/monitors/icamera/`
- drop-in 配置 → `/usr/share/wireplumber/wireplumber.conf.d/51-icamera.conf`

**注意**: `51-icamera.conf` 默认会**禁用 v4l2 和 libcamera 视频相机 monitor**，
让桌面只看到 icamera 源。如果你的系统有其他需要 v4l2/libcamera 相机的应用,
请编辑该文件, 把 `main` profile 里的禁用项去掉。

### 3) 重启 WirePlumber

```sh
systemctl --user restart wireplumber
```

---

## 使用

### 查看相机节点

```sh
pw-cli ls Node | grep icamera
```

或

```sh
pw-dump 2>/dev/null | python3 -c '
import json,sys
for o in json.load(sys.stdin):
    if o.get("type")=="PipeWire:Interface:Node":
        print(o["info"]["props"]["node.name"])
'
```

典型输出:

```
icamera_gc5035_rear
icamera_ov5675_front
```

### 用 GStreamer 预览

```sh
gst-launch-1.0 pipewiresrc path=icamera_gc5035_rear ! videoconvert ! autovideosink
```

---

## 3A 参数控制

3A 参数（曝光 / 增益 / AWB / 帧率）既可以**启动时通过节点属性**配置, 也可以**运行时动态修改**。

### 启动时配置（节点属性）

在 `monitors/icamera/enumerate-device.lua` 的节点属性里加如下键即可：

| 属性 | 含义 | 单位 |
|------|------|------|
| `icamera.exposure` | 曝光时间 | 纳秒 (ns) |
| `icamera.exposure-us` | 曝光时间（便捷） | 微秒 (µs) |
| `icamera.gain` | 灵敏度增益 | ISO (float) |
| `icamera.ae-mode` | AE 模式（0=auto, 非0=manual） | int |
| `icamera.awb-mode` | AWB 模式 | int |
| `icamera.awb-r-gain` / `-g-` / `-b-` | 手动 AWB 增益 | int |
| `icamera.frame-rate` | 帧率 | fps (float) |
| `icamera.3a-cadence` | 3A cadence | int |

### 运行时动态修改（不重启）

节点暴露了一个 `Props` 参数通道 `params`，结构为
`Struct((String:key, Pod:value)*)`，键与上表一致（前缀 `api.icamera.*`）。

用 `pw-cli` 在 streaming 中直接改 gain（**注意: 顶层不要有 "Props" 包装**，
`params` 用平铺数组）：

```sh
# 运行时把 gain 改为 400 (ISO)
ID=$(pw-dump 2>/dev/null | python3 -c '
import json,sys
for o in json.load(sys.stdin):
    if o.get("type")=="PipeWire:Interface:Node" and \
       o["info"]["props"]["node.name"]=="icamera_gc5035_rear":
        print(o["id"]); break
')
pw-cli s $ID Props '{"params": ["api.icamera.gain", 400.0]}'
```

当前支持的 `api.icamera.*` 键:

| key | Pod 类型 |
|-----|----------|
| `api.icamera.ae-mode` | Int |
| `api.icamera.exposure` | **Long** (ns) |
| `api.icamera.gain` | Float |
| `api.icamera.awb-mode` | Int |
| `api.icamera.awb-r-gain` / `-g-` / `-b-` | Int |
| `api.icamera.frame-rate` | Float |
| `api.icamera.3a-cadence` | Int |

> **注意**: spa-json 的自动类型推断会把裸整数当成 `Int` 而非 `Long`。
> `api.icamera.exposure` 需要 **Long**，因此用 `pw-cli` 的纯 JSON 无法直接设置它；
> `gain` / `frame-rate` (Float) 和整数类键则可以直接设。要设 exposure 需用客户端
> 通过 `spa_pod_builder` 显式构造 `Long`。

---

## 调试

插件内的运行时诊断（streaming 统计、warmup 丢帧、buffer 复用、3A 更新等）统一通过
**标准 `spa_log` 通道**输出，因此会进入 PipeWire / journald，并可按组件与级别过滤
（`SPA_DEBUG` / `PW_LOG`）：

```sh
# 实时看插件日志（info 及以上）
journalctl --user -f | grep -iE "icamera|camhal"

# 想看到帧级 debug 诊断（warmup/统计/EMIT 等），开 debug 级别：
SPA_DEBUG=4 systemctl --user restart wireplumber
```

关键生命周期与 3A 更新为 `info` 级别，帧级高频统计为 `debug` 级别，`debug` 默认关闭。

---

## 路线图（下一步）

当前处于"功能主线已跑通"状态, 仍有以下高优先级项待完善:

- [ ] **P0 — Stride 处理**: 目前 `data_size` 与 buffer stride 按 `width` 估算,
      未使用 `camhal_backend_configure()` 返回的 HAL 真实 stride。若 HAL 对某些
      分辨率（如 RGB-IR 全分辨率）加行 padding, 帧会产生错位/绿条。需把 HAL stride
      贯通到 format/buffers 协商并按 stride 逐行拷贝。
- [x] **P0 — 调试日志收敛**: 已把所有 `fprintf(stderr)` 诊断收敛为 `spa_log` 分级
      （info = 生命周期/3A 更新, debug = 帧级高频统计），可通过 `SPA_DEBUG` 过滤。
- [ ] **P0→P1 — 帧率联动**: `EnumFormat`/`Format` 当前写死 `30fps`, 未与 3A
      `frame-rate` / HAL 实际 fps 联动。
- [ ] **P1 — 多种像素格式**: 目前仅 NV12。支持 BGRx 等格式 / 转换, 以便 RGB-IR
      交叉验证。
- [ ] **P1 — 时钟/时序对齐**: `SPA_NODE_FLAG_RT` 但采集线程非 RT 线程, 无自定义
      Clock 提供。
- [ ] **P1 — Props/PropInfo 枚举完整性**: 端口 PropInfo 标为 READ 但未实现完整描述。
- [ ] **P1 — HAL metadata 透传**: 曝光/3A statistics 等 metadata 尚未透传到管线。

---

## 许可证

[MIT](LICENSE)
