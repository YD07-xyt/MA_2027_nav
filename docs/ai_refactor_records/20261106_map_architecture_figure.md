# 地图算法架构图 改造记录

## User Intent

用户要求把地图算法架构按开源文档插图的风格画成简洁美观的图，写入 `docs/架构图`，并参考 `docs/nav/开源参考/` 中技术报告的插图与版式。

## Scope

仅新增文档与图：

- `docs/架构图/map_architecture.svg`
- `docs/架构图/map_architecture.png`
- `docs/架构图/README.md`
- 本记录

不修改任何源码、配置、topic、frame、launch。

## Out of Scope

不修改地图算法与参数，不运行构建、节点、rosbag，不改变任何运行行为。

## Explorer Findings

### Files inspected

- [ma_map.hpp](</home/xyt/map/src/MA_2027_nav/src/map/include/map/ma_map.hpp>)
- [map.yaml](</home/xyt/map/src/MA_2027_nav/config/map.yaml>)
- [01-概率占据建图与滑窗](</home/xyt/map/src/MA_2027_nav/docs/nav/算法思路/map/01-概率占据建图与滑窗.md>)
- [02-地形与隧道语义](</home/xyt/map/src/MA_2027_nav/docs/nav/算法思路/map/02-地形与隧道语义.md>)
- [03-ESDF距离场与避障梯度](</home/xyt/map/src/MA_2027_nav/docs/nav/算法思路/map/03-ESDF距离场与避障梯度.md>)
- 参考报告插图：`docs/nav/开源参考/` 三份报告，以及用户提供的 Super-LIO Fig. 2 版式。

### Active logic path

`/lio/cloud_world` + `/lio/base_odom` → `MaMap::update_cloud` → `ROGMap::updateProbMap`（概率占据/滑窗/遗忘）→ `GroundLayer` + `TerrainAnalyzer`（地面/高度场/可通行）→ 2D 动态层 ∪ 先验 → `GridMap`（占用 + 有符号 EDT）→ 规划器快照。

### Data flow

点云/位姿 → 3D 概率占据 → 2.5D 地面/地形 → 2D 栅格 + 距离场；先验 PGM 与隧道语义作为数据注入 2D 层。

### Risk notes

图为逻辑归纳，不是逐行代码映射；参数以 `config/map.yaml` 当前值为准。图为示意资产，不参与编译。

### Recommended modification boundary

只新增 `docs/架构图/` 下的图与说明，以及本记录。

## Modifier Changes

### Files changed

1. `docs/架构图/map_architecture.svg`：1720×620 手写 SVG，横版流水线 + 右侧 3D→2D 示意图 + 图注。
2. `docs/架构图/map_architecture.png`：Chrome headless 2× 渲染预览。
3. `docs/架构图/README.md`：图、说明、风格参考与内容依据。
4. 本记录。

### Key changes

新增地图算法架构图与说明文档，不改运行代码。

### Behavior preserved

无运行行为改动。

### Behavior intentionally adjusted

无。

### Notes

渲染链路：Chrome headless `--force-device-scale-factor=2` 截图；SVG 已通过 XML 解析检查。

## Auditor Review

### Checks performed

- [x] 关键路径检查：图示内容与 `ma_map.hpp`、`map.yaml` 对照
- [x] diff 检查：仅新增 `docs/架构图/` 与本记录
- [x] grep / 静态检查：SVG XML 解析通过
- [x] launch / yaml / XML 检查：未改动
- [x] 用户允许范围内的测试或静态检查：PNG 渲染并目视检查
- [ ] 如需构建，已取得用户明确许可：未构建

### Issues found

无。

### Final result

PASS
