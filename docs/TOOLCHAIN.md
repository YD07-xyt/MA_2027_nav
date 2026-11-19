# 静态工具链

工具根目录由 `scripts/` 自身位置解析，只面向本仓库。依赖 Python >= 3.10、PyYAML、clang-format 18、Git；当前环境验证为 Python 3.10、PyYAML 5.4.1、clang-format 18.1.8。构建工具不参与静态入口。

## 命令

```bash
./scripts/verify.sh
./scripts/verify.sh --static-only --report .cache/verify/latest.json
./scripts/verify.sh --full-scan --report .cache/verify/full.json
python3 scripts/check_boundaries.py
bash scripts/format.sh src/明确文件.cpp
```

默认/`--static-only`：全量语法和边界扫描；边界与现有基线比较；格式按 HEAD 到工作区变化的行检查，新文件/重命名后路径检查完整文件。包含已暂存、未暂存和未跟踪源码。无源码改动会明确保留存量问题，而不是对全仓格式作成功声明。

`--full-scan`：检查全部一方 C++ 格式，全部启发式发现参与失败判定。返回 0 表示声明范围通过，1 表示检查/工具失败，2 表示参数错误；没有构建或动态验证。

`check_boundaries.py`：单独输出完整 JSON 初筛结果；存在语法错误或任一启发式发现时返回非零，不使用基线豁免。

格式修复只接受明确指定的 `src/` C++ 文件，检查所有路径后再写入。所有工具拒绝父路径越界和扫描范围中的符号链接，排除第三方、构建、安装、日志、缓存。不扫描父目录项目。

## 扫描范围与覆盖限制

| 检查 | 当前覆盖 | 限制 |
| :--- | :--- | :--- |
| YAML | `config/` 与 `src/` 的 yaml/yml，config 中 rviz 也作 YAML 解析 | 不检查 yaml-cpp 数值不变式与资源可用性 |
| Python/XML | `src/`、`launch/` 中 Python AST/XML 及 package.xml；工具脚本 AST | 不导入/执行 launch，不验证 ROS 运行 |
| C++/CMake | `src/` 一方源码/头、各 CMake 和根 CMake | 核心 ROS 名称、公开重型 include、部分 Eigen auto 正则初筛，可能误报/漏报 |
| 格式 | NAV 现有配置，clang-format replacement 与改动行相交 | 存量未修改文件的格式仅由 full-scan 检查 |
| 工具测试 | 隔离 NAV 夹具与假命令，验证路径、错误拒绝、只读和失败传播 | 不代表 NAV C++ 功能测试 |

存量初筛基线在 `scripts/static_baseline.json`，以规则+相对路径+代码摘要和次数匹配，移动行号不生成新豁免；新增重复条目仍失败。当前基线来源为首次迁移前 NAV revision；条目是待审查候选，不自动认定为 bug，特别是测试公开头和返回具体类型的函数可能触发初筛。

仅在用户批准登记存量问题时显式执行：

```bash
./scripts/verify.sh --write-baseline "本次登记的理由"
```

该操作写基线，不是验证通过；语法问题不能豁免，也不建立格式豁免。更新后审查基线差异与理由。默认验证只读源码和 Git，报告仅在显式提供 `--report` 时写 `.cache/verify/`；测试夹具使用临时目录。

## 历史文档工具

迁移开始时已核查 `docs/tools/`，最终检查时该目录的 9 个已跟踪文件处于删除状态；删除不来自本次迁移补丁，保持当前现场。以下仅记录基线版本中的工具，当前工作区不能直接调用；新静态入口不依赖它们。

基线中的 `extract_params.py`、`extract_launch.py`、`extract_ros2.py` 为按需静态证据提取器；若后续重新引入，应显式限定本仓库 root、输入及产物目录，并验证路径和失败传播。`verify_docs.py`、`test_injection.py` 还依赖缺失的 docs/nav 与 docs/_evidence 等输入。`cpp/inventory` 是需授权编译、运行时依赖 clangd 的独立 C++ 工具。以上均未接入本轮门禁。

## 离线基准与对比工具

核心管线离线基准目标为 `src/test/src/bench_nav_pipeline.cpp`（CMake 目标 `bench_nav_pipeline`），依赖系统已安装的 Google Benchmark、OpenCV、Eigen3、yaml-cpp、PCL 与 OSQP/OsqpEigen，不依赖 ROS。只读输入为 `images/rmuc2024.png`、`config/map.yaml`、`config/planner.yaml`，默认零仓库副作用（报告显式输出到被 `.gitignore` 忽略的 `.cache/bench/`）。

```bash
# 仅增量编译基准目标（依赖已有 build/ma_nav）
cmake --build build/ma_nav --target bench_nav_pipeline -j

# 仅跑正确性自检（零基准计时）
./build/ma_nav/src/test/bench_nav_pipeline --sanity-only

# 采集改前/改后 JSON（可用 --benchmark_filter 按前缀筛选：GridMap|MaMap|PathPlanning|TrajOptimize|FsmReplan|Controller）
./build/ma_nav/src/test/bench_nav_pipeline \
  --benchmark_repetitions=5 \
  --benchmark_out=.cache/bench/before.json \
  --benchmark_out_format=json

# 只读对比（默认比较 real_time median，阈值 +/-5%；--fail-on-regression 在超阈值回退时返回 1）
python3 scripts/compare_bench.py .cache/bench/before.json .cache/bench/after.json
python3 scripts/compare_bench.py --metric cpu_time --threshold 5.0 --fail-on-regression \
  .cache/bench/before.json .cache/bench/after.json
```

`scripts/compare_bench.py` 仅依赖 Python 3.10+ 标准库，默认只读（仅在显式提供 `--json-out` 时写文件），自动归一化 `ns/us/ms/s`、检查环境上下文差异（`library_build_type`/`host_name`/`num_cpus`/`cpu_scaling_enabled`）并拒绝 `error_occurred` 样本；其隔离单元测试位于 `scripts/tests/test_compare_bench.py`，随 `./scripts/verify.sh` 一并执行。在开启 CPU 变频或大小核（P/E 核）调度的机器上，单线程微基准与并发锁争用项（如 `Controller/Mpc_Solve`、`MaMap/ConcurrentGetSnapshot`）可能出现 5%~9% 的调度抖动；精细对比时建议对单线程项加 `taskset -c 0` 绑核或先确认 CPU 频点一致，并在报告中结合 `cv` 与上下文警告解读。

限制：离线基准覆盖核心 C++ 模块（`GridMap`、`MaMap`、`PathPlanning`、`TrajOptimize`、`FsmReplan`、`Controller`），不替代 ROS 节点回调、rosbag 回放、TSan/ASan 并发插桩或实机验证。

## 后续模式

`./scripts/verify.sh` 保持纯静态，不执行构建或动态基准；全量构建预设、clang-tidy、ASan/UBSan、TSan 尚未接入统一入口。它们需要独立确认 CMake、输入、数据库和测试条件。当前静态脚本不会因为传入扩展选项而生成代码或构建。
