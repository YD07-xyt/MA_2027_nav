# MA_Nav    

## 开发工作流

唯一工作区为本仓库 `/home/xyt/map/src/MA_2027_nav`。先读 [AGENTS.md](AGENTS.md) 和 [开发工作流](docs/WORKFLOW.md)；接口变化读取 [接口契约](docs/rules/interfaces.md)。

```bash
./scripts/verify.sh                 # 默认增量静态门禁，无构建
./scripts/verify.sh --full-scan     # 全量现状扫描，存量问题也返回失败
./scripts/verify.sh --static-only --report .cache/verify/latest.json
bash scripts/format.sh src/明确文件.cpp  # 显式格式修复，使用现有风格

# 核心代码改前/改后离线基准对比（流程见 docs/WORKFLOW.md 与 docs/TOOLCHAIN.md）
cmake --build build/ma_nav --target bench_nav_pipeline -j
./build/ma_nav/src/test/bench_nav_pipeline --benchmark_repetitions=5 \
  --benchmark_out=.cache/bench/before.json --benchmark_out_format=json
python3 scripts/compare_bench.py .cache/bench/before.json .cache/bench/after.json
```

依赖 Python 3.10+、PyYAML、clang-format 18、Git。范围与返回码见 [工具链](docs/TOOLCHAIN.md)，当前能力见 [架构索引](docs/ARCHITECTURE.md) 和 [规范缺口](docs/STANDARD_GAPS.md)。静态通过不代表构建或功能测试通过；除 [AGENTS.md](AGENTS.md) 约定的 `bench_nav_pipeline` 离线基准常驻授权外，以下构建和安装命令供人工使用，Agent 执行仍须单独授权。

## env


```sh
    sudo apt update
    sudo apt install cpufrequtils
    sudo apt install libompl-dev
```

mpc  
osqp osqpEigen

qpOASES
```bash
git clone https://github.com/coin-or/qpOASES.git
cd ./qpOASES
mkdir build && cd ./build
cmake ..
make 
sudo make install
```

OsqpEigen

```bash
git clone https://github.com/gbionics/osqp-eigen.git
cd ./osqp-eigen
mkdir build && cd ./build
cmake ..
make 
sudo make install
```


casadi(不必须)
```sh
    sudo apt-get install swig
    git clone https://github.com/casadi/casadi.git
    cd casadi
    mkdir build
    cd build
    cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON _DWITH_MUMPS=ON ..
    make
    sudo make install
```

ros2
```sh
wget http://fishros.com/install -O fishros && . fishros 
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```


## build
```sh
colcon build
```
or
```sh
colcon build --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
```


## run

```sh
source install/setup.sh
ros2 launch ma_nav run.launch.py
```
建图or导航
planner.yaml中的mapping_model 更改


保存服务的话题在planner.yaml中  save_map_srv_topic save_tunnel_regions_srv_topic'

建图保存地图
```sh
ros2 service call /save_global_map std_srvs/srv/Trigger "{}"
```

保存地图隧道语义
```sh
ros2 service call /save_tunnel_regions std_srvs/srv/Trigger "{}"
```

可视化
```sh
rviz2 -d ./nav.rviz
```
