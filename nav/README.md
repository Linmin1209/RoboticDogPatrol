# Robotic Dog Patrol Navigation System

## 环境配置安装指南

本指南将帮助您配置Robotic Dog Patrol导航系统的完整开发环境。

### 系统要求

- Ubuntu 22.04 LTS (推荐)
- Python 3.8+
- 至少8GB RAM
- 支持ROS2 Humble的硬件

## 1. 安装ROS2 Humble

### 1.1 设置软件源

```bash
# 添加ROS2 GPG密钥
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加ROS2仓库
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.2 安装ROS2 Humble

```bash
# 更新软件包列表
sudo apt update

# 安装ROS2 Humble Desktop版本（包含GUI工具）
sudo apt install ros-humble-desktop

# 或者安装ROS2 Humble Base版本（仅命令行工具）
# sudo apt install ros-humble-ros-base
```

### 1.3 配置环境

```bash
# 将ROS2环境配置添加到~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 验证安装
ros2 --help
```

## 2. 安装Unitree ROS2官方库

### 2.1 克隆Unitree ROS2仓库

```bash
# 创建workspace目录
mkdir -p ~/unitree_ws/src
cd ~/unitree_ws/src

# 克隆Unitree ROS2官方仓库
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

### 2.2 安装依赖

```bash
# 安装ROS2依赖包
sudo apt install ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-geometry-msgs
sudo apt install ros-humble-nav-msgs ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# 安装编译工具
sudo apt install build-essential cmake python3-colcon-common-extensions
```

### 2.3 编译Unitree ROS2

```bash
cd ~/unitree_ws
colcon build --symlink-install
```

## 3. 配置网络设置

### 3.1 修改setup.sh文件

找到Unitree ROS2包中的setup.sh文件，通常位于：
```bash
# 编辑setup.sh文件
nano ~/unitree_ws/src/unitree_ros2/setup.sh
```

### 3.2 配置网卡参数

在setup.sh文件中，修改对应网卡的参数配置



### 3.3 应用网络配置

```bash
# 使配置生效
source ~/unitree_ws/src/unitree_ros2/setup.sh
```

## 4. 验证ROS2 Topic通信

### 4.1 检查Topic信息

```bash
# 终端2：查看可用的话题
ros2 topic list
```

## 5. 编译项目库

### 5.1 编译unitree_go库

```bash
# 进入项目根目录
cd /path/to/RoboticDogPatrol

# 编译unitree_go
colcon build --packages-select unitree_go --symlink-install
```

### 5.2 编译unitree_interfaces库

```bash
# 编译unitree_interfaces
colcon build --packages-select unitree_interfaces --symlink-install
```

### 5.3 Source安装文件

```bash
# 使编译的包生效
source install/setup.bash

# 或者添加到~/.bashrc中永久生效
echo "source /path/to/RoboticDogPatrol/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. 安装Python依赖

### 6.1 安装导航系统依赖

```bash
# 进入nav目录
cd nav

# 安装Python依赖
pip3 install -r requirements.txt
```


## 7. 运行导航测试

### 7.1 启动导航示例

```bash
# 确保ROS2环境已激活
source /opt/ros/humble/setup.bash
source /path/to/RoboticDogPatrol/install/setup.bash

# 运行导航示例
python3 nav/navigator_example.py
```

### 7.2 交互式控制

运行后，您可以使用以下键盘命令进行交互：

#### 📋 基础导航命令
- `M` - 开始/结束建图
- `N` - 在当前位姿添加节点
- `NM` - 手动添加节点（指定坐标）
- `E` - 在节点间添加边
- `CN` - 关闭所有节点
- `S` - 开始导航
- `P` - 暂停导航
- `R` - 恢复导航
- `I` - 初始化位姿
- `L` - 开始重定位并收集节点和边数据
- `FP` - 定点导航
- `NS` - 获取导航状态

#### 🗑️ 数据管理
- `C` - 清除环境点云
- `T` - 清除轨迹点云
- `A` - 清除所有点云
- `V` - 保存环境点云
- `R` - 保存轨迹点云
- `S` - 保存组合点云

#### 📍 位姿和状态
- `G` - 获取当前位姿
- `RT` - 检查实时位姿状态
- `I` - 获取点云信息
- `D` - 设置下采样参数

#### 🗑️ 节点/边管理
- `DN` - 删除节点
- `DE` - 删除边
- `DAN` - 删除所有节点
- `DAE` - 删除所有边
- `QN` - 查询节点
- `QE` - 查询边
- `NO` - 网络概览（完整网络分析）
- `PA` - 路径分析（查找节点间路径）

#### 💾 内部存储管理
- `PN` - 发布所有节点（从内部存储）
- `PE` - 发布所有边（从内部存储）
- `PNE` - 发布所有节点和边（从内部存储）
- `CI` - 清除内部存储
- `GI` - 获取内部存储信息

#### 🔄 自动收集（Demo B2 风格）
- `AC` - 在当前位姿自动收集节点
- `CS` - 收集并保存所有节点/边
- `AL` - 自动收集循环
- `PC` - 准备收集
- `CM` - 清除并开始建图
- `GS` - 获取收集状态

#### 📢 通知缓存控制
- `NC` - 配置通知缓存
- `NS` - 获取通知缓存状态
- `NT` - 测试通知缓存

#### 📷 相机控制
- `CA` - 相机控制菜单

#### 🎬 可视化控制
- `VS` - 开始可视化
- `VT` - 停止可视化
- `VH` - 检查可视化状态

#### ❓ 帮助和退出
- `H` - 显示帮助
- `Q` - 退出

**导航流程**：

1. **开启建图**
   ```
   Enter command: M
   Start (S) or End (E) mapping? S
   ```

2. **遥控移动机器狗**
   - 使用遥控器控制机器狗在建图区域内移动
   - 确保机器狗能够完整覆盖需要导航的区域
   - 观察建图质量，确保地图完整

3. **关闭建图**
   ```
   Enter command: M
   Start (S) or End (E) mapping? E
   ```

4. **返回原位并开启重定位收集模式**
   ```
   Enter command: L
   # 系统会自动：
   # - 删除所有现有节点和边
   # - 开始重定位
   # - 开始导航
   # - 初始化位姿
   # - 开始收集节点和边数据
   ```

5. **收集导航节点和边**
   
   - 手动使用命令 `N` `E` 在关键位置收集节点
  

6. **上传所有节点和边**
   ```
   Enter command: PNE
   # 将内部存储的所有节点和边发布到系统
   ```

7. **关闭所有节点**
   ```
   Enter command: CN
   # 关闭所有节点，准备开始导航
   ```

8. **返回原位置并开启导航模式**
   ```
   Enter command: S
   # 开始循环导航模式
   ```

### 第二阶段：导航执行

1. **循环导航模式**
   ```
   Enter command: S
   # 开始按照预设路径进行循环导航
   # 机器狗会自动在收集的节点间进行导航
   ```

2. **导航控制**
   - `P` - 暂停导航
   - `R` - 恢复导航
   - `P` 后使用 `CN` - 退出导航

3. **定点导航（可选）**
   ```
   Enter command: FP
   # 输入目标坐标进行定点导航(未实现，但可以输入目标的编入地图的节点)
   ```

4. **状态查询**
   ```
   Enter command: G
   # 获取当前位姿信息
   ```

## 8. API 接口说明

### 8.1 核心导航功能

#### 建图相关
- `start_mapping(attribute: int = 0) -> int` - 开始建图
- `end_mapping(floor_index: int = 0, pcdmap_index: int = 0) -> int` - 结束建图

#### 导航相关
- `start_navigation() -> int` - 开始导航
- `start_single_navigation(node_id: int) -> int` - 开始单点导航
- `default_navigation_loop() -> int` - 开始默认导航循环
- `pause_navigation() -> int` - 暂停导航
- `recover_navigation() -> int` - 恢复导航
- `go_home() -> int` - 返回原点
- `navigate_to_point(x: float, y: float, yaw: float = 0.0, goal_node_id: int = None, map_name: str = "default") -> bool` - 定点导航

#### 位姿相关
- `pose_init(translation: Tuple[float, float, float] = (0.0, 0.0, 0.0), quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> int` - 初始化位姿
- `start_relocation() -> int` - 开始重定位
- `get_current_pose() -> Optional[dict]` - 获取当前位姿
- `get_realtime_pose() -> Optional[dict]` - 获取实时位姿
- `wait_for_fresh_pose(timeout: float = 2.0) -> Optional[dict]` - 等待新的位姿数据

### 8.2 节点和边管理

#### 节点操作
- `add_node(node_name: int, x: float, y: float, z: float = 0.0, yaw: float = 1.57, publish_immediately: bool = False) -> int` - 添加节点
- `add_node_at_current_pose(node_name: int, use_realtime: bool = True, publish_immediately: bool = False) -> tuple[bool, int]` - 在当前位姿添加节点
- `delete_node(node_ids: List[int]) -> int` - 删除节点
- `delete_all_nodes() -> int` - 删除所有节点
- `close_all_nodes() -> int` - 关闭所有节点

#### 边操作
- `add_edge(edge_name: int, start_node: int, end_node: int, dog_speed: float = 1.0, seq: str = "index:123;", publish_immediately: bool = False) -> int` - 添加边
- `delete_edge(edge_ids: List[int]) -> int` - 删除边
- `delete_all_edges() -> int` - 删除所有边

#### 查询功能
- `query_node(attribute: int = 1) -> tuple[bool, int]` - 查询节点
- `query_edge(attribute: int = 2) -> tuple[bool, int]` - 查询边

### 8.3 内部存储管理

#### 内部存储操作
- `publish_all_nodes() -> int` - 发布所有内部存储的节点
- `publish_all_edges() -> int` - 发布所有内部存储的边
- `publish_all_nodes_and_edges() -> tuple[int, int]` - 发布所有节点和边
- `clear_internal_nodes() -> None` - 清除内部节点存储
- `clear_internal_edges() -> None` - 清除内部边存储
- `clear_internal_storage() -> None` - 清除所有内部存储
- `get_internal_nodes() -> dict` - 获取内部节点信息
- `get_internal_edges() -> dict` - 获取内部边信息
- `get_internal_storage_info() -> dict` - 获取内部存储信息
- `remove_internal_node(node_id: int) -> bool` - 移除内部节点
- `remove_internal_edge(edge_id: int) -> bool` - 移除内部边

### 8.4 自动收集功能（Demo B2 风格）

#### 自动收集操作
- `add_node_at_current_pose_auto_collect(node_name: int = None, auto_connect: bool = True) -> tuple[bool, int]` - 自动收集节点
- `collect_and_save_nodes_edges(clear_after_save: bool = True) -> tuple[int, int]` - 收集并保存节点和边
- `auto_collect_loop(node_interval: float = 2.0, max_nodes: int = 10) -> None` - 自动收集循环
- `get_collection_status() -> dict` - 获取收集状态
- `clear_collection_and_start_mapping() -> None` - 清除收集并开始建图
- `prepare_for_collection() -> None` - 准备收集

### 8.5 通知缓存控制

#### 通知缓存操作
- `_cache_notice_data(duration: float = 2.0) -> dict` - 缓存通知数据
- `get_notice_cache_status() -> dict` - 获取通知缓存状态
- `configure_notice_cache(cache_duration: float = 2.0, auto_cache: bool = True) -> dict` - 配置通知缓存
- `test_notice_cache(duration: float = 3.0) -> dict` - 测试通知缓存

### 8.6 点云和可视化

#### 点云操作
- `clear_accumulated_cloud() -> None` - 清除累积点云
- `clear_trajectory_cloud() -> None` - 清除轨迹点云
- `clear_all_clouds() -> None` - 清除所有点云
- `save_accumulated_cloud(filename: str = "accumulated_cloud.pcd") -> None` - 保存累积点云
- `save_trajectory_cloud(filename: str = "trajectory_cloud.pcd") -> None` - 保存轨迹点云
- `save_combined_cloud(filename: str = "combined_map.pcd") -> None` - 保存组合点云
- `get_cloud_size() -> int` - 获取点云大小
- `get_trajectory_size() -> int` - 获取轨迹大小
- `get_total_cloud_size() -> int` - 获取总点云大小
- `set_downsample_parameters(max_size: int = 100000, voxel_size: float = 0.05) -> None` - 设置下采样参数

#### 可视化操作
- `start_visualization() -> None` - 开始可视化
- `stop_visualization() -> None` - 停止可视化
- `is_visualization_running() -> bool` - 检查可视化状态

### 8.7 相机控制

#### 相机操作
- `get_camera_data(camera_name: List[str], camera_mode: Optional[List[str]] = None) -> Dict` - 获取相机数据
- `get_latest_frame(front=True)` - 获取最新帧
- `show_front_camera()` - 显示前置相机
- `show_back_camera()` - 显示后置相机
- `start_camera_display(front=True, back=False)` - 开始相机显示

### 8.8 系统状态

#### 状态查询
- `get_nav_state() -> Dict` - 获取导航状态
- `get_last_notice() -> Optional[dict]` - 获取最后通知
- `wait_for_command_confirmation(seq_id: str, timeout: float = 5.0) -> Optional[dict]` - 等待命令确认
- `get_command_confirmation(seq_id: str) -> Optional[dict]` - 获取命令确认
- `get_latest_notice_analysis() -> dict` - 获取最新通知分析
- `debug_notice_topic(duration: float = 10.0) -> None` - 调试通知话题

### 8.9 自动导航设置

#### 自动导航配置
- `set_auto_nav(map_name: str = 'default', area: list = None, path_point: list = None) -> bool` - 设置自动导航
- `start_navigation_loop(seq: str = "index:123;") -> bool` - 开始导航循环

### 8.10 系统控制

#### 系统操作
- `shutdown()` - 关闭系统
- `clear_command_confirmations() -> None` - 清除命令确认
