# WSL 环境搭建与运行指南

## 1. 环境要求

| 组件 | 说明 |
|---|---|
| Windows 10/11 | 建议 Windows 11，自带 WSLg 图形支持 |
| WSL2 | Ubuntu 22.04 |
| ROS 2 Humble | 完整桌面安装 |
| RViz2 | 可视化（可选） |

## 2. 安装 WSL 与 ROS 2

### 2.1 安装 WSL2 + Ubuntu 22.04

以管理员身份打开 PowerShell：

```powershell
wsl --install -d Ubuntu-22.04
```

安装完成后重启电脑，首次进入 Ubuntu 会提示创建用户名和密码。

### 2.2 安装 ROS 2 Humble

进入 WSL 终端：

```bash
# 添加 ROS 2 源
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS 2 Humble 桌面版
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2.3 安装项目依赖

```bash
sudo apt install -y \
  ros-humble-angles \
  ros-humble-diagnostic-updater \
  ros-humble-sophus \
  ros-humble-pcl-conversions \
  ros-humble-geographic-msgs \
  ros-humble-rviz2 \
  libgeographic-dev \
  libpcl-dev \
  libtbb-dev \
  libyaml-cpp-dev \
  robin-map-dev
```

## 3. 配置 WSL 内存上限

`robot_localization` 包编译时内存消耗较大，建议给 WSL 分配至少 4GB 上限（动态占用，不影响日常使用）。

在 Windows 文件资源管理器地址栏输入 `%USERPROFILE%`，新建文件 `.wslconfig`，写入：

```ini
[wsl2]
memory=4GB
```

保存后重启 WSL：

```powershell
wsl --shutdown
```

## 4. 克隆与构建

### 4.1 克隆项目

```bash
git clone --recurse-submodules https://github.com/starry1N/WUTA.git
cd WUTA
```

### 4.2 一键构建并启动

```bash
# 清理构建 + 全量编译 + 启动仿真 + RViz 可视化
./start_simulator.sh --clean --rviz
```

首次编译约需 5-10 分钟。之后日常使用只需：

```bash
./start_simulator.sh --skip-build --rviz
```

## 5. 启动参数

| 参数 | 作用 |
|---|---|
| `--clean` | 清理后重新编译 |
| `--build-only` | 只编译，不启动 |
| `--skip-build` | 跳过编译，直接启动 |
| `--rviz` | 启动 RViz2 可视化 |
| `launch_fsd:=false` | 只启动模拟器，不启动算法链 |
| `track_file:=skidpad` | 选择赛道 (trackdrive/skidpad/acceleration) |
| `mission_mode:=skidpad` | 选择比赛模式 |

示例：

```bash
# 只跑模拟器，不跑算法
./start_simulator.sh --skip-build --rviz launch_fsd:=false

# 跑 Skidpad 模式
./start_simulator.sh --skip-build track_file:=skidpad mission_mode:=skidpad
```

## 6. 验证运行

启动后另开一个 WSL 终端，检查数据是否正常产出：

```bash
source /opt/ros/humble/setup.bash
source ~/WUTA/WUTA-FSD/ros2_ws/install/setup.bash
source ~/WUTA/WUTA-SIM/install/setup.bash

# 检查 LiDAR 点云频率
ros2 topic hz /hesai/pandar

# 检查所有话题
ros2 topic list
```

## 7. 常见问题

### 编译时 OOM (Killed signal)

确认 `.wslconfig` 已配置，或手动限制编译线程：

```bash
cd ~/WUTA/WUTA-FSD/ros2_ws
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j1" colcon build --packages-select robot_localization
```

### RViz 无法弹出

确认 WSL 版本支持图形界面：

```bash
echo $DISPLAY
```

如果为空，更新 WSL：

```powershell
wsl --update
```

### CRLF 换行符错误

如果见到 `SyntaxError` 或 `command not found`，修复换行符：

```bash
find . -name "*.py" -o -name "*.sh" | xargs sed -i 's/\r$//'
```

### 项目在 Windows 文件系统下编译失败

WSL 中 `/mnt/c/` 路径不支持符号链接，必须把项目放到 WSL 原生文件系统：

```bash
cp -r /mnt/c/path/to/WUTA ~/WUTA
```
