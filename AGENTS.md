# 项目 Agent 行为准则

适用于本仓库的 AI Agent、Codex、Claude、dsh 及 skill。这里是比赛代码，所有修改以**可复现、最小改动、可审计**为原则。

本规范约束后续改动；现有工程偏差按 [规范缺口](docs/STANDARD_GAPS.md) 登记，不将规范接入或更新解释为批量整改授权。

## 1. 工作区与授权边界

- 唯一修改工作区为 `/home/xyt/map/src/MA_2027_nav`。工具以本仓库为根，不自动发现父目录或其他项目。用户明确指定的外部参考路径可按需只读查阅，不在其中修改文件或执行工具链。
- **构建必须先取得用户明确许可**：包括 CMake 配置/探测、编译、链接、代码生成及 `colcon build`、`catkin build`、`catkin_make`、`make`、`ninja`、`cargo build/check`、`npm/yarn build` 等。CMake 配置也可能编译探测程序，不属于静态检查。不确定命令是否触发这些操作时先询问。
- **离线基准常驻授权（例外）**：修改 `src/map/**`、`src/planner/**`、`src/utils/**` 或 `src/test/src/bench_nav_pipeline.cpp` 前后，已常驻授权直接执行 `bench_nav_pipeline` 增量编译（`cmake --build build/ma_nav --target bench_nav_pipeline -j`）、离线运行 `./build/ma_nav/src/test/bench_nav_pipeline`（报告仅写 `.cache/bench/`）与只读对比脚本 `python3 scripts/compare_bench.py`。若 `build/ma_nav` 不存在或需重新 `colcon build`/首次 CMake 配置，仍须先取得构建许可。其他编译目标、ROS 节点/launch、回放/实机运行仍须单独授权，Git 写操作限制不变。
- 无需构建授权的检查包括只读搜索、Git 状态/差异、Python AST、XML/YAML 解析及本仓库的静态验证入口与隔离工具测试。除上述常驻离线基准外，其他导航离线程序、ROS 节点/launch、回放或实机运行须获得相应运行范围的明确授权；构建许可不自动包含运行许可。
- **Agent 只修改工作区文件，不执行版本控制写操作**：禁止更改 Git 索引、历史、远端、分支、标签或通过 Git 命令覆盖工作区文件。包括 `git add/commit/push/tag/merge/rebase/cherry-pick/revert`、`git reset`、`git checkout`、`git restore`、`git clean`、分支写操作、`gh pr merge`、`gh release create` 等；用户授权构建不改变此限制。只读 `git status/diff/log/show` 等允许使用。
- **禁止 writingplans 及类似计划说明工具**，不生成独立计划文件。需求与边界明确后直接执行，必要的简短说明不阻塞修改，事后补充审计记录。
- **只有用户明确要求时才启动子 Agent**；复杂度本身不构成授权。角色与交接按 [工作流](docs/WORKFLOW.md#5-用户明确要求时的多-agent-流程) 执行。

## 2. 事实、契约与修改边界

1. 先查实际入口、配置、调用方和工作区差异，区分生效、离线、备用与历史逻辑，保留已有用户修改。
2. 修改前明确模块、目标、允许范围、行为变化与验收范围。代码能回答的问题先查代码；单位、坐标系、时钟、失败或容量策略仍有待决项时，只暂停依赖该决策的实现并询问用户。
3. 新增公开接口或改变调用方可观察契约时，按 [接口规则](docs/rules/interfaces.md) 提出最小草案与调用方影响，确认后实现；用户已明确确认的内容不重复询问，既有契约内的局部实现直接执行。
4. 优先最小修改与局部修补。未经明确要求，不进行无关重构、批量格式化、重命名、删除历史代码/注释分支，或引入大量变量、函数、类和新框架。
5. 保留比赛验证过的主逻辑。未经明确要求，不改变策略优先级、模块边界、topic、frame、blackboard key、参数名/默认值、launch 组合、QoS、timer 或 callback group。不为“理论更优”改动现有方案。
6. 沿周边命名与风格实现；抽象应有实际需求，避免重复校验已保证的不变式，不添加高频 debug 或无用统计。已有边界校验与兼容逻辑不借此删除。
7. 不通过放宽检查配置、添加无依据的抑制、删除测试或擅自重写基线来让检查通过。门禁变化需属于明确任务范围，存量登记遵守 [工具链](docs/TOOLCHAIN.md)。

## 3. 按需读取

| 触发条件 | 必读文件 |
| :--- | :--- |
| 修改、审计或执行验证 | [docs/WORKFLOW.md](docs/WORKFLOW.md)：执行顺序、记录与交付 |
| 修改 `src/**` | [src/AGENTS.md](src/AGENTS.md)；修改 `src/ros2/**` 另读 [src/ros2/AGENTS.md](src/ros2/AGENTS.md) |
| 新增公开接口或改变可观察契约 | [docs/rules/interfaces.md](docs/rules/interfaces.md)：状态、失败、所有权、时间与性能条件 |
| 修改 YAML、参数加载或配置路径 | [docs/rules/yaml_params.md](docs/rules/yaml_params.md) |
| 修改模块依赖、启动链路或通信关系 | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| 使用验证工具或声明工程能力 | [docs/TOOLCHAIN.md](docs/TOOLCHAIN.md)、[docs/STANDARD_GAPS.md](docs/STANDARD_GAPS.md) |

专项文件补充本文件，不放宽授权与比赛逻辑限制；遇到无法同时满足的要求时，说明冲突并询问用户。

## 4. 验证与交付底线

- `./scripts/verify.sh` 默认仅做静态检查；参数不授予构建或运行许可。静态通过不代表编译、导航运行、并发、性能或实机验证通过。
- **核心代码改前/改后基准对比（必选）**：修改 `src/map/**`、`src/planner/**`、`src/utils/**` 的算法、数据结构、并发、内存或核心逻辑前，须先用 `bench_nav_pipeline` 采集改前基线（`.cache/bench/<task>_before.json`），改动后以同参数重编重跑采集改后结果（`.cache/bench/<task>_after.json`），并用 `scripts/compare_bench.py` 输出前后对比；执行步骤与例外见 [工作流](docs/WORKFLOW.md#3-修改与验证)。仅文档/注释/纯格式/静态脚本修改标记基准对比 `N/A`。
- 修改完成后检查本次 diff 与受影响链路；记录触发条件、模板和交付格式统一见 [工作流](docs/WORKFLOW.md#4-审查记录与交付)。普通问答无需生成记录或套用修改交付模板。
- 结果必须带范围。逐项使用 `PASS`（通过）、`NEEDS_FIX`（明确失败/缺陷）、`NOT_VERIFIED`（未验证）、`N/A`（不适用），不把缺失证据写成通过。
- 修改交付中若未获构建许可（且未触发离线基准常驻授权），明确写：**未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。**
