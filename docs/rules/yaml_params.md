# YAML 与配置路径规则

修改 YAML、加载器、ROS 路径参数前读取本文件。

- 权威配置来自 YAML，由 yaml-cpp 直接加载；ROS 使用现有必填路径参数 `planner_params_path`，不为同一算法参数再维护一套 ROS 默认值。
- 当前 `run.launch.py` 默认指向包 share 中 `config/planner.yaml`；显式覆盖路径保持原行为。嵌套的地图、保存目录等路径按实际使用处记录基准，不假定相对 YAML 文件解析。
- 现有 `planner.yaml` 中仍有 `src/navigation/...` 历史相对路径；工具链迁移仅登记，不改比赛配置。后续修改路径先确认部署方式和相对路径语义。
- 必填、默认值、数值范围、缺失/非法处理分开定义。新增加载不得静默猜值；已有 fallback 的变动须按根规范确认兼容性。
- 参数修改同步核对 C++ 读取、单位/坐标系注释与文档，不保留重复键。静态扫描覆盖 `config/**/*.yaml/yml/rviz` 和 `src/**/*.yaml/yml`。
- `./scripts/verify.sh` 检查语法和重复键；它不证明 yaml-cpp 业务加载成功、路径文件可用或参数满足数值不变式。
