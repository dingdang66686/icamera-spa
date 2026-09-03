-- WirePlumber icamera monitor: enumerate-device
-- 从 CamHAL 动态枚举相机，代替硬编码相机表。
--
-- WirePlumber 的 Lua 沙箱屏蔽了 io.popen / os.execute（只有受限的 os.getenv），
-- 因此不能 spawn 辅助程序。这里通过 package.cpath 加载我们随 icamera-spa 提供的
-- 原生 Lua C 模块 "camhal"（/usr/lib/lua/5.5/camhal.so），它直接链接 libcamhal，
-- 暴露 camhal.discover() -> { {name=, facing=, camera_id=, description=}, ... }。
--
-- 为每个真实内置相机创建一个 icamera SPA source node。

log = Log.open_topic ("s-monitors-icamera")

function createIcameraNode (properties)
  local source = Plugin.find ("standard-event-source")
  local e = source:call ("create-event", "create-icamera-node", nil, nil)
  e:set_data ("node-properties", properties)
  e:set_data ("factory", "api.icamera.source")
  EventDispatcher.push_event (e)
end

-- 通过原生 camhal Lua C 模块动态发现相机。
-- 每项: {name, facing, camera_id, description}，facing: 0=rear 1=front。
local function discoverCameras ()
  local ok, camhal = pcall (require, "camhal")
  if not ok then
    log:warning ("monitor/icamera: failed to load camhal module: " .. tostring (camhal))
    return {}
  end

  local found = camhal.discover ()
  if type (found) ~= "table" then
    log:warning ("monitor/icamera: camhal.discover() returned unexpected value")
    return {}
  end

  local cameras = {}
  for _, c in ipairs (found) do
    if type (c.name) == "string" and c.name ~= "" then
      -- 去掉 HAL 的 "-uf"(user-facing) 后缀用于节点名；再用 facing 决定 rear/front
      local base = c.name:gsub ("%-uf$", "")
      local suffix = (c.facing == 0) and "rear" or "front"
      local nodeN = "icamera_" .. base .. "_" .. suffix
      local desc = (type (c.description) == "string" and c.description ~= "")
                       and c.description or c.name
      cameras[#cameras + 1] = {
        name     = nodeN,
        desc     = desc,
        device   = c.name,
        facing   = c.facing,
        priority = (c.facing == 0) and 750 or 800,
      }
    end
  end
  return cameras
end

local cameras = discoverCameras ()
log:info ("monitor/icamera: discovered " .. #cameras .. " camera(s)")

for i, cam in ipairs (cameras) do
  local properties = {
    ["node.name"]        = cam.name,
    ["node.description"] = cam.desc,
    ["node.nick"]        = cam.desc,
    ["device.name"]      = cam.device,
    ["object.path"]      = cam.name,   -- 关键：GStreamer/portal 用 object.path 定位节点
    ["media.class"]      = "Video/Source",
    -- media.type 是标准 Video/Source 节点必需的属性。Firefox/OBS/pipewiresrc 等
    -- 客户端按 media.class=Video/Source + media.type=Video 来发现和链接视频源；
    -- 缺 *media.type* 会导致它们在 PipeWire 里枚举不到此相机（gst target-not-found
    -- 同源问题）。必须显式设为 Video。
    ["media.type"]       = "Video",
    ["media.role"]       = "Camera",
    ["device.api"]       = "icamera",
    ["node.pause-on-idle"] = false,
    ["priority.session"] = cam.priority,
    ["factory.name"]     = "api.icamera.source",
    -- front/back: Snapshot(aperture) uses api.libcamera.location to decide
    -- whether to mirror the preview.  rear(0)=back(no mirror), front(1)=front
    -- (mirror).  Missing => treated as front => wrongly mirrors the rear.
    ["api.libcamera.location"] = (cam.facing == 0) and "back" or "front",
  }
  createIcameraNode (properties)
  log:info ("pushed icamera node event for " .. cam.device .. " (" .. cam.name .. ")")
end
