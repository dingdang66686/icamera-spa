-- WirePlumber icamera monitor: create-node
-- 通过 spa-node-factory 创建 api.icamera.source 节点。
-- 用模块级局部表持有节点引用, 防止节点被生命周期结束/GC 立即销毁。

cutils = require("common-utils")

log = Log.open_topic("s-monitors-icamera")

config = {}
config.rules = Conf.get_section_as_json("monitor.icamera.rules", Json.Array {})

-- 模块级局部表: 在脚本 chunk 存活期间一直持有节点引用, 避免 LocalNode 被销毁
local icamera_nodes = {}

SimpleEventHook {
  name = "monitor/icamera/create-node",
  interests = {
    EventInterest {
      Constraint { "event.type", "=", "create-icamera-node" },
    },
  },
  execute = function(event)
    local properties = event:get_data("node-properties")

    if cutils.parseBool(properties["node.disabled"]) then
      log:notice("icamera node " .. properties["node.name"] .. " disabled")
      return
    end

    log:info("creating icamera node " .. tostring(properties["node.name"]))
    local node = LocalNode("spa-node-factory", properties)
    node:activate(Feature.Proxy.BOUND)

    -- 保存引用, 防止被生命周期/GC 销毁
    icamera_nodes[properties["node.name"]] = node
  end
}:register()
