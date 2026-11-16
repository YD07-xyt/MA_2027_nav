# ROS 适配与编排规则

当前 `GlobalPlanner2d` 同时承担订阅、地图/规划/控制调度、状态维护与发布，尚非完整薄壳。迁移工具链不授权业务搬迁。

- 配置链路为 launch 的 `planner_params_path` → ROS 入口 → yaml-cpp。保持该名称，算法参数由 YAML 加载；新增配置遵守 `docs/rules/yaml_params.md`。
- 修改 Msg 与核心类型转换时，核对坐标系、单位、事件时间/接收时间及消息有效性；现有外部约定与已实现校验分开记录。
- 回调借用消息及捕获对象须说明生命周期；跨组共享数据核对同步。保留现有 QoS、timer 和 callback group，变更需明确确认。
- 新算法优先放核心层；当前规划状态、折叠策略、急停与进度等编排若要抽离，先明确契约和调用方影响。
- 使用 `./scripts/verify.sh` 做静态初筛。编译、clang-tidy 数据库生成及 ROS 运行验证仍须按根规范授权。
