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

- `M` - 开始/结束建图
- `N` - 在当前位姿添加节点
- `E` - 在节点间添加边
- `S` - 开始导航
- `P` - 暂停导航
- `R` - 恢复导航
- `I` - 初始化位姿
- `L` - 开始重定位
- `C` - 清除环境点云
- `T` - 清除轨迹点云
- `A` - 清除所有点云
- `V` - 保存环境点云
- `G` - 获取当前位姿
- `H` - 显示帮助
- `Q` - 退出


**步骤**：

1. **开始建图**
   ```
   Enter command: M
   Start (S) or End (E) mapping? S
   ```

2. **创建导航节点**
   - 移动到需要设置节点的位置
   - 使用命令 `N` 创建节点
   - 系统会自动获取当前位置作为节点坐标

3. **创建导航边**
   - 使用命令 `E` 创建边
   - 指定起始节点和结束节点
   - 设置导航速度参数

### 第二阶段：导航执行

1. **预设路线导航**
   ```
   Enter command: S
   # 开始按照预设路径导航
   ```

2. **单点导航**
   ```
   Enter command: G
   # 获取当前位姿信息
   # 然后导航到指定节点
   ```
