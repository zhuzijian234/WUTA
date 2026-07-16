# 仿真器 Skidpad 与 Acceleration 模式 Bug 排查与修复

## 概述
目前情况:
1 八字环绕和直线加速大致路径正确,且主要问题不在感知层,是规划和控制层
2 八字环绕从右圈转换到左圈时转弯力度不够
3 三个赛道都没有刹车逻辑,八字环绕最后会一直绕左圈转圈,直线加速一直走
4 赛道循迹轨迹仍然不贴和锥桶,但是呈现一定规律

| Bug | 现象 | 影响范围 | 严重度 |
|-----|------|----------|--------|
| A | 路径每 0.1s 跟着车跑，车辆永远追不上终点 | Skidpad / Acceleration | 🔴 致命 |
| B | `findTargetIndex` 反向搜索在八字自交路径上跨圈跳变，车辆原地转圈 | Skidpad（自交路径） | 🔴 致命 |
| C | RViz 中看不到规划路径的青色 LINE_STRIP | Skidpad / Acceleration | 🟡 中 |
| D | 路径走完后车辆不停，在终点无限绕圈 | Skidpad / Acceleration | 🟡 中 |
| E | Trackdrive 模式走到 centerline 末端误停车 | Trackdrive | 🟡 中 |
| F | 近零误差时转向放大逻辑丢失符号，方向盘抖动 | 所有模式 | 🟢 低 |
| G | `target_idx_` 无越界保护，waypoints 被替换时越界跳变 | 所有模式 | 🟡 中 |

---

## Bug A：路径原点每 0.1 秒跟着车跑

### 现象

Skidpad 八字或 Acceleration 直线路径在 RViz 中持续漂移，车辆永远无法走完预定轨迹。

### 根因

**触发源**：[`simulation_bridge.py`](../WUTA-SIM/simulator_bringup/simulator_bringup/simulation_bridge.py) 以 10Hz 持续发布 `MissionState(state=EXPLORE)`：

```python
self.status_timer = self.create_timer(0.1, self._publish_status)
```

**病灶**：[`path_generator_node.cpp`](../WUTA-FSD/ros2_ws/src/planning/path_generator/src/path_generator_node.cpp) 的 `onMissionState()` 每次回调都重新调用路径生成函数：

```cpp
if (mission_mode_ == State::MISSION_SKIDPAD) {
    auto lane = generateSkidpadPath();      // ← 每次用 current_pose_ 重新算
    waypoints_pub_->publish(lane);
}
```

而路径生成函数从 `current_pose_`（车辆当前位置）取原点：

```cpp
const double cx = current_pose_.pose.position.x;  // 车当前位置作为圆心
const double cy = current_pose_.pose.position.y;
```

**完整因果链**：

```
simulation_bridge 每 0.1s 发 MissionState (10Hz)
  → path_generator::onMissionState() 每 0.1s 触发
    → generateSkidpadPath() 从 current_pose_ 生成路径
      → 车辆在移动 → current_pose_ 每 0.1s 改变
        → 八字圆心每 0.1s 漂移 → 路径永远在追赶移动的原点
          → 车追不上路径，乱跑
```

### 修复

路径改为**首次生成后缓存**，后续 10Hz 触发仅更新时间戳并重发：

```cpp
// 头文件新增
autoware_msgs::msg::Lane cached_lane_;
bool path_generated_{false};

// onMissionState() 中
if (mission_mode_ == State::MISSION_SKIDPAD) {
    if (!path_generated_) {
        cached_lane_ = generateSkidpadPath();  // 仅首次从初始位置生成
        path_generated_ = true;
    }
    auto lane = cached_lane_;
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
}
```

检测到 `mission_mode` 变更时使缓存失效，确保模式切换后重新生成。

### 为什么 Trackdrive 不受影响？

Trackdrive 的路径由 `boundary_detector` 从锥桶地图（map 固定坐标系）实时计算中线，坐标不随车辆移动，且 `onCenterline()` 只透传不做坐标偏移。

---

## Bug B：`findTargetIndex` 反向搜索在自交路径上致命跳变

### 现象

Skidpad 模式下车辆在某个点开始原地转圈，不跟随八字锥桶轨迹。日志显示 `target_idx_` 在路径不同圈之间随机跳变。

### 根因

**这是所有 Bug 中最严重的一个。** 原始 `findTargetIndex` 采用"反向搜索"策略：

```cpp
// 原始算法：从路径末尾向前扫描
for (int i = static_cast<int>(waypoints.size()) - 1; i >= 1; --i) {
    const double d = planeDist(waypoints[i], car);
    if (d < ld) return i;   // 返回"最后一个在 LD 内"的 waypoint
}
return waypoints.size() - 1;
```

**为什么 Trackdrive 上碰巧能工作？**

Trackdrive 路径是一条不自交的曲线，每个 (x,y) 坐标只出现一次。反向搜索总是找到"路径上最远且在 LD 范围内"的 waypoint，行为正确。

**为什么 Skidpad 上彻底失败？**

Skidpad 路径是八字形（figure-8），在 (0,0) 处自交，**6 个 waypoint 共用同一物理坐标**：

```
       右圈 (wp0 - wp144)
      ╭───╮
      │   │
      ╰─╮─╯
        │  (0,0) ← wp0, wp72, wp144, wp145, wp217, wp289 都在这里
      ╭─╯─╮
      │   │
      ╰───╯
       左圈 (wp145 - wp289)
```

反向搜索**每次从 wp289 开始**。车在 (0,0) 附近时，wp289 距离 ≈ 0 < LD，立即返回 289。target 直接跳到终点，车还没起步就被告知"你已经到了"。

`compute()` 中 `dist < 1e-6` 导致 `valid=false`，不发送控制指令。车辆随惯性漂移，但只要稍微离开 (0,0)，反向搜索又立刻锁定 wp289。**target 永远卡在 289，车在 (0,0) 附近来回绕圈。**

更隐蔽的情况：车在八字中途时，6 个 (0,0) 位置的 waypoint 都在 LD 范围内，**浮点精度决定命中哪一个**。这就是日志中 `target=289→254→232→289` 随机跳变的根因。

### 修复

**完全去掉反向搜索和最近点查找，改为纯正向单调搜索：**

```cpp
int PurePursuit::findTargetIndex(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double ld)
{
  const int N = static_cast<int>(waypoints.size());
  if (N == 0) return -1;

  // 越界保护：waypoints 被替换成更小向量时 clamp
  if (target_idx_ >= N) {
    target_idx_ = N - 1;
  }

  // 纯正向搜索：从上次 target 前 5 个点开始向前扫
  const int search_start = std::max(0, target_idx_ - 5);

  for (int i = search_start; i < N; ++i) {
    const double d = planeDist(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y);
    if (d >= ld) return i;   // 第一个"够远"的点
  }

  // 所有剩余点都在 LD 内 → 瞄准终点
  return N - 1;
}
```

**核心区别：不找最近点，只向前看。**

```
原始（反向搜索）：              修复后（正向单调搜索）：

wp289 ←── 从这往回扫           wp45 ──→ wp46 ──→ ... → wp55 ✓
  │                              ↑
  │  所有 d<LD 的都跳过           从上次 target-5 开始
  ↓                             只向前看，永不回头
命中 wp144（错了！这是另一圈）
                              target: 45→55→65→...→289 严格单调递增
target: 289→144→250→... 乱跳
```

即使路径在 (0,0) 自交 100 次，搜索总是从当前位置向前找，不可能跳到另一圈的同坐标点。`target_idx_` **只能增大，永不减小。**

---

## Bug C：路径可视化 LINE_STRIP 从未发布

### 现象

RViz 中看不到青色规划路径线，无法判断路径是否正确生成。

### 根因

原始 `path_generator_node` **完全没有可视化发布器**。`onMissionState()` 只发布 `/planning/final_waypoints`（给 controller），不发布 Marker。

### 修复

新增 `viz_pub_` 发布器和 `publishVisualization()` 方法：

```cpp
void PathGeneratorNode::publishVisualization(
  const autoware_msgs::msg::Lane & lane, float r, float g, float b)
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker line;
  line.header = lane.header;   // 使用 lane 的 header（frame_id="map"）
  line.ns     = "planned_path";
  line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  line.scale.x = 0.08;
  line.color.r = r; line.color.g = g; line.color.b = b;
  line.color.a = 0.9f;

  for (const auto & wp : lane.waypoints) {
    line.points.push_back(wp.pose.pose.position);
  }
  arr.markers.push_back(line);
  viz_pub_->publish(arr);
}
```

三种模式用不同颜色：Skidpad 青色、Acceleration 橙色、Trackdrive 绿色。

同时新增 `trajectory_viz_pub_` 发布器，在 `onPose()` 中累积车辆行驶轨迹并以金色 LINE_STRIP 发布。

---

## Bug D：路径走完后车辆不停

### 现象

Skidpad/Acceleration 路径走完后，`target_idx_` 卡在最后一个 waypoint，`compute()` 中 `dist < 1e-6` 导致不发指令。车辆靠惯性在终点附近无限绕圈。

### 根因

原始 controller 没有"路径终点"的概念。Pure Pursuit 假定路径无限循环或被持续更新（如 Trackdrive 的 centerline），但 Skidpad/Acceleration 是**静态有限路径**——走完就结束。

### 修复

在 `controlLoop()` 中增加终点停车逻辑：

```cpp
// 仅在固定路径模式下启用（Skidpad / Acceleration）
const bool is_fixed_path =
  (mission_mode_ == MissionState::MISSION_SKIDPAD ||
   mission_mode_ == MissionState::MISSION_ACCELERATION);

if (is_fixed_path) {
  const int N = static_cast<int>(waypoints_.size());
  if (pure_pursuit_->targetIndex() >= N - 3) {
    const auto & last_wp = waypoints_.back();
    const double dx = vehicle_state_.x - last_wp.pose.pose.position.x;
    const double dy = vehicle_state_.y - last_wp.pose.pose.position.y;
    if (std::sqrt(dx*dx + dy*dy) < 1.0) {
      raw_cmd.velocity   = 0.0;
      raw_cmd.steering_angle = 0.0;
    }
  }
}
```

条件：target 已到末尾 3 个 waypoint 以内 **且** 车距离终点 < 1m。两个条件同时满足才停车，防止提前误停。

---

## Bug E：Trackdrive 模式走到 centerline 末端误停车

### 现象

Trackdrive 模式下车辆走到某个点后突然不动。

### 根因

Bug D 的停车逻辑最初没有区分 mission_mode，对所有模式生效。Trackdrive 的 centerline 是 `boundary_detector` 动态生成的——只覆盖当前检测到的锥桶范围，不是完整赛道。车到达当前 centerline 末端时，停车逻辑误判为"到达终点"。

### 修复

同 Bug D 的 `is_fixed_path` 条件——停车逻辑仅在 Skidpad 和 Acceleration 模式下生效。Trackdrive 的 centerline 会随车辆前进持续更新，"末端"只是暂时的，不应停车。

---

## Bug F：近零误差时转向放大逻辑丢失符号

### 现象

车辆在直线路段方向盘微幅来回抖动。

### 根因

原始 `compute()` 中的"数值稳定化"代码：

```cpp
double numerator = 2.0 * x_body;
if (std::abs(numerator) < 0.1) {
    numerator = 10.0 * std::copysign(1.0, numerator) * numerator;
}
```

`std::copysign(1.0, x) * x` 等价于 `|x|`（取绝对值）。乘以 10 后 **符号信息丢失**——无论目标在左还是在右，都朝同一方向转。且标准 Pure Pursuit 公式 `kappa = 2·x_body / dist²` 本就稳定，不需要人为放大。

### 修复

移除放大逻辑，使用标准公式：

```cpp
const double numerator = 2.0 * x_body;
```

---

## Bug G：`target_idx_` 无越界保护

### 现象

日志中 `target_idx_` 偶尔从正常值跳变到不相关的位置，随后逐渐恢复。

### 根因

`controller_node` 的 `waypoints_` 在两个线程间无同步保护：
- **订阅回调 `onWaypoints()`**：接收新路径，替换 `waypoints_`
- **定时器回调 `controlLoop()`**：读取 `waypoints_`，调用 `findTargetIndex`

当 waypoints 被替换成不同大小的向量时（如启动瞬间收到 trackdrive 的 centerline），旧的 `target_idx_` 可能 >= 新 N。正向搜索的 `search_start = target_idx_ - 5 >= N`，for 循环不执行，直接 `return N - 1`——此时 N-1 指向完全不同的路径位置。

### 修复

在 `findTargetIndex` 开头增加越界 clamp（见 Bug B 修复代码）。虽然不能完全解决无锁的数据竞争，但防止了最坏情况下的越界访问和 target 跳变。

---

## 修改文件清单

| 文件 | 修改内容 |
|------|----------|
| `planning/path_generator/include/path_generator/path_generator_node.hpp` | 新增 `cached_lane_`、`path_generated_`、`trajectory_`、`viz_pub_`、`trajectory_viz_pub_` 成员 |
| `planning/path_generator/src/path_generator_node.cpp` | 路径缓存、可视化发布、行驶轨迹累积、路径方向修正 |
| `control/controller/include/controller/pure_pursuit.hpp` | `findTargetIndex` 去 `const`（需修改 `target_idx_`） |
| `control/controller/src/pure_pursuit.cpp` | 纯正向单调搜索 + 越界保护；移除有符号丢失的数值放大 |
| `control/controller/include/controller/controller_node.hpp` | 新增 `mission_mode_` 成员变量 |
| `control/controller/src/controller_node.cpp` | 终点停车（仅固定路径）+ mission_mode 追踪 + 调试日志 |
| `simulator_bringup/simulation_bridge.py` | 新增 `/localization/velocity` 发布（冗余但无害） |

---

## 关键设计原则

1. **自交路径上禁止用最近点查找**。任何依赖"距离最近 waypoint"的算法在 figure-8 等自交路径上都会因为同坐标多 waypoint 而失败。应使用严格单调的 forward search。

2. **固定路径和动态路径要区别对待**。Skidpad/Acceleration 路径是静态的、有限长的——需要终点停车。Trackdrive 路径是动态的、持续更新的——不能停车。

3. **ROS 2 订阅回调和定时器在不同线程**。对共享数据（如 `waypoints_`）的读写需要同步保护，或至少做防御性越界检查。

4. **路径坐标系必须一致**。`generateSkidpadPath()` 从 `current_pose_`（map 坐标系）取原点，路径本身也在 map 坐标系。可视化 Marker 的 `header.frame_id` 必须为 `"map"` 才能在 RViz 中正确叠加。
