# simulator_bringup

`simulator_bringup` 是 WUTA 仿真系统的统一 ROS 2 启动包。各模拟器仍是独立包；
本包通过包含它们各自的 launch 文件进行编排，并可选启动 WUTA-FSD Level A
闭环。`ins_simulator` 目前只预留接入口，不包含任何实现或包依赖。

## Dependency order

1. `vehicle_model` 先启动，接收 WUTA-FSD 的
   `autoware_msgs/msg/Command`，发布 `/sim/ground_truth`。
2. `can_simulator` 和 `lidar_sim` 在 ground truth 源启动后再启动。
3. `ins_simulator` 的位置已经预留；当前不查找、不启动该包。
4. 启用 `launch_fsd` 时，WUTA-FSD 按数据流顺序启动：
   `lidar_detection` -> `cone_map_builder` -> `boundary_detector` ->
   `path_generator` -> `controller`.

`simulation_bridge` 为 Level A 联调提供 `/localization/pose`、
`map -> base_link` TF、就绪状态以及（`auto_start:=true` 时）`EXPLORE`
任务状态。它只是 ground truth 接口适配器，不实现 INS 模型。

## Build

推荐从仓库根目录使用一键脚本。它会先调用 WUTA-FSD 自带的
`ros2_ws/build_ws.sh` 完整构建 16 个 FSD 包，再构建模拟器 overlay：

```bash
cd /path/to/WUTA
./start_simulator.sh
```

### 一键脚本用法

| 参数 | 作用 |
|---|---|
| 无参数 | 增量构建完整 WUTA-FSD 和模拟器，然后启动完整闭环 |
| `--clean` | 清理两个工作区后重新完整构建并启动 |
| `--build-only` | 完成构建后退出，不启动 ROS 节点 |
| `--skip-build` | 使用已有安装空间直接启动 |
| `--rviz` | 启动时同时打开 RViz2 默认可视化配置 |
| `-h` / `--help` | 显示脚本帮助 |
| `--` | 后续参数全部原样传给 ROS launch |

构建和启动示例：

```bash
# 默认：增量构建并启动模拟器和 WUTA-FSD
./start_simulator.sh

# 默认闭环，并同时打开 RViz2
./start_simulator.sh --rviz

# 清理两个工作区，完整重建后启动
./start_simulator.sh --clean

# 只构建，不启动
./start_simulator.sh --build-only

# 清理后只构建，用于验证完整构建
./start_simulator.sh --clean --build-only

# 使用已有构建结果启动完整闭环
./start_simulator.sh --skip-build

# 使用已有构建结果启动完整闭环，并打开 RViz2
./start_simulator.sh --skip-build --rviz

# 只启动模拟器，不启动 WUTA-FSD 算法链
./start_simulator.sh --skip-build launch_fsd:=false

# 选择赛道和任务模式
./start_simulator.sh track_file:=skidpad mission_mode:=skidpad

# 调整依赖阶段之间的启动间隔
./start_simulator.sh startup_delay:=1.0

# 自定义车辆初始位姿
./start_simulator.sh -- \
  track_file:=/path/to/track.yaml \
  start_x:=1.0 start_y:=2.0 start_yaw:=0.5
```

手动构建时，必须先完整构建并加载 WUTA-FSD，再构建模拟器 overlay。这样
`vehicle_model` 才能找到 `autoware_msgs`：

```bash
cd /path/to/WUTA/WUTA-FSD/ros2_ws
./build_ws.sh
source install/setup.bash

cd ../../WUTA-SIM
colcon build --base-paths . --symlink-install \
  --packages-up-to simulator_bringup
source install/setup.bash
```

## Run

```bash
ros2 launch simulator_bringup simulator.launch.py
```

Useful overrides:

```bash
ros2 launch simulator_bringup simulator.launch.py launch_fsd:=false
ros2 launch simulator_bringup simulator.launch.py launch_rviz:=true
ros2 launch simulator_bringup simulator.launch.py track_file:=skidpad mission_mode:=skidpad
ros2 launch simulator_bringup simulator.launch.py startup_delay:=1.0
ros2 launch simulator_bringup simulator.launch.py launch_ins:=true  # 仅打印预留提示
ros2 launch simulator_bringup simulator.launch.py \
  track_file:=/path/to/track.yaml start_x:=1.0 start_y:=2.0 start_yaw:=0.5
```

`track_file` 和 `mission_mode` 应选择同一比赛项目。若赛道起点不是原点，还需传入
一致的 `start_x`、`start_y` 和 `start_yaw`。

未来接入 INS 时，在 launch 文件标出的 `INS integration point` 处包含
`ins_simulator` 自身的 launch 文件，并在 `package.xml` 增加对应
`exec_depend` 即可。

## RViz2 visualization

推荐直接用一键脚本启动完整闭环和 RViz2：

```bash
cd /path/to/WUTA
./start_simulator.sh --rviz
```

若已经构建完成，可跳过构建：

```bash
cd /path/to/WUTA
./start_simulator.sh --skip-build --rviz
```

该命令等价于启动 `simulator_bringup` 时传入 `launch_rviz:=true`，并加载默认
RViz 配置：

```bash
ros2 launch simulator_bringup simulator.launch.py launch_rviz:=true
```

默认配置文件安装在：

```text
share/simulator_bringup/rviz/wuta_simulator.rviz
```

源码路径为：

```text
WUTA-SIM/simulator_bringup/rviz/wuta_simulator.rviz
```

默认 RViz 设置：

| Display | Topic | 用途 |
|---|---|---|
| `TF` | `map -> base_link -> lidar` | 坐标系关系 |
| `Odometry` | `/sim/ground_truth` | 车辆真值位置 |
| `PointCloud2` | `/hesai/pandar` | LiDAR 仿真点云 |
| `MarkerArray` | `/sim/lidar/visible_cones` | LiDAR 当前可见锥筒 |
| `MarkerArray` | `/sim/lidar/track_cones` | 从赛道 YAML 读取的全量锥筒地图 |
| `MarkerArray` | `/perception/lidar/cones_viz` | 感知检测锥筒 |
| `MarkerArray` | `/mapping/cone_map_viz` | 建图后的全局锥筒地图 |
| `MarkerArray` | `/planning/centerline_viz` | 规划中心线 |
| `MarkerArray` | `/control/target_viz` | 控制目标/预瞄点 |

RViz 的 `Fixed Frame` 已配置为 `map`。`/hesai/pandar` 点云已配置为
`Best Effort` QoS，以匹配传感器数据发布方式。

只可视化模拟器、不启动 WUTA-FSD 算法链时：

```bash
./start_simulator.sh --skip-build --rviz launch_fsd:=false
```

此时可见的主要 topic 是 `/sim/ground_truth`、`/hesai/pandar` 和
`/sim/lidar/visible_cones`、`/sim/lidar/track_cones`；感知、建图、规划和控制相关
可视化 topic 不会发布。

也可以手动启动 RViz2：

```bash
source /opt/ros/humble/setup.bash
source /path/to/WUTA/WUTA-FSD/ros2_ws/install/setup.bash
source /path/to/WUTA/WUTA-SIM/install/setup.bash
rviz2 -d /path/to/WUTA/WUTA-SIM/install/simulator_bringup/share/simulator_bringup/rviz/wuta_simulator.rviz
```

常用检查命令：

```bash
ros2 topic list
ros2 topic hz /hesai/pandar
ros2 topic hz /perception/lidar/cones
ros2 topic hz /mapping/cone_map
ros2 topic hz /planning/centerline
ros2 run tf2_tools view_frames
```

如果 RViz 提示 `No transform from [lidar] to [map]`，先确认仿真仍在运行，并等待
`simulation_bridge` 发布 `map -> base_link`，以及静态 TF 发布
`base_link -> lidar`。如果只缺点云显示，检查 `/hesai/pandar` Display 的
`Reliability Policy` 是否为 `Best Effort`。
