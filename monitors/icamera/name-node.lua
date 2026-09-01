-- WirePlumber icamera monitor: name-node
-- 补全 node.name / factory.name 等节点属性。

log = Log.open_topic("s-monitors-icamera")

SimpleEventHook {
  name = "monitor/icamera/name-node",
  interests = {
    EventInterest {
      Constraint { "event.type", "=", "create-icamera-node" },
    },
  },
  execute = function(event)
    local properties = event:get_data("node-properties")
    local factory = event:get_data("factory")

    properties["factory.name"] = factory
    properties["node.pause-on-idle"] = false

    if not properties["node.name"] then
      properties["node.name"] = "icamera-source"
    end
    if not properties["node.description"] then
      properties["node.description"] = "ICamera Source"
    end

    event:set_data("node-properties", properties)
  end
}:register()
