# 机器人狗导航系统使用说明

## 概述

本导航系统采用**节点式导航**模式，通过在建图过程中同步建立导航节点，然后创建节点之间的边来定义导航路径，最终实现预设路线的自动导航功能。

## 系统架构

### 核心组件

1. **Navigator类** (`navigator.py`)
   - 提供完整的导航控制接口
   - 管理节点和边的创建
   - 处理实时位姿数据
   - 支持点云可视化
   - 集成相机数据获取功能
   - 支持机器人状态监控

2. **InteractiveNavigator类** (`navigator_example.py`)
   - 提供交互式命令行界面
   - 支持实时命令输入
   - 集成可视化功能

3. **HTTPNavigatorAgent类** (`http_navigator_agent.py`)
   - 提供HTTP REST API接口
   - 支持远程网络控制
   - 包含完整的API文档
   - 支持日志记录功能

## 导航模式

### 1. 节点式导航（主要模式）

**工作原理**：
- 在建图过程中，在关键位置创建导航节点
- 通过边连接相邻节点，形成导航路径
- 导航时按照预设的节点序列进行移动

**优势**：
- 路径规划精确可控
- 支持复杂环境导航
- 可预设速度和安全参数
- 便于调试和优化

### 2. 单点导航模式

**功能**：
- 直接导航到指定的单个节点
- 适用于简单任务和测试
- 支持实时位姿获取

## 使用流程

### 第一阶段：建图与节点创建

```bash
# 启动导航程序
python3 navigator_example.py
```

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

## 详细命令说明

### 建图控制
- `M` - 开始/结束建图
- `I` - 初始化位姿
- `L` - 开始重定位

### 节点管理
- `N` - 在当前位姿添加节点
- `G` - 获取当前位姿信息
- `RT` - 检查实时位姿状态

### 边管理
- `E` - 添加边（连接两个节点）
- `NR` - 添加带限制检查的节点
- `ER` - 添加带限制检查的边

### 导航控制
- `S` - 开始导航
- `P` - 暂停导航
- `R` - 恢复导航

### 数据管理
- `C` - 清除环境点云
- `T` - 清除轨迹点云
- `A` - 清除所有点云
- `V` - 保存环境点云
- `R` - 保存轨迹点云
- `S` - 保存组合点云

### 可视化控制
- `VS` - 启动可视化
- `VT` - 停止可视化
- `VH` - 检查可视化状态

## 实时位姿功能

### 解决的问题
- ROS2节点处理消息的时延问题
- 确保 `add_node` 时获取准确的实时位姿

### 新增功能
1. **实时位姿检查** (`get_realtime_pose()`)
   - 检查位姿数据是否在超时范围内（默认0.5秒）
   - 如果数据过期，返回 `None`

2. **等待新鲜位姿数据** (`wait_for_fresh_pose()`)
   - 等待新的位姿数据到达
   - 可设置超时时间

3. **增强的节点添加功能**
   - 支持实时位姿模式
   - 可选择使用实时位姿或任何可用位姿

### 使用方法
```python
# 使用实时位姿数据添加节点
navigator.add_node_at_current_pose(node_name, use_realtime=True)

# 检查实时位姿状态
navigator.get_realtime_pose()

# 等待新鲜位姿数据
fresh_pose = navigator.wait_for_fresh_pose(timeout=2.0)
```

## 配置参数

### 位姿相关
- `pose_timeout`: 位姿数据超时时间（默认0.5秒）
- `last_pose_update_time`: 最后位姿更新时间戳

### 点云相关
- `max_cloud_size`: 点云最大点数（默认100000）
- `downsample_voxel_size`: 下采样体素大小（默认0.05）

## 使用示例

### 示例1：创建简单导航路径

```bash
# 1. 启动程序
python3 navigator_example.py

# 2. 开始建图
Enter command: M
Start (S) or End (E) mapping? S

# 3. 移动到第一个位置，创建节点1
Enter command: N
Enter node name/ID (default: 1): 1

# 4. 移动到第二个位置，创建节点2
Enter command: N
Enter node name/ID (default: 2): 2

# 5. 创建从节点1到节点2的边
Enter command: E
Enter start node ID (default: 1): 1
Enter end node ID (default: 2): 2
Enter edge name/ID (default: 1): 1
Enter dog speed (default: 1.0): 1.0

# 6. 开始导航
Enter command: S
```

### 示例2：检查实时位姿

```bash
# 检查当前位姿状态
Enter command: RT

# 输出示例：
# 🔄 Checking realtime pose status...
# ✅ Fresh pose data available
# 📍 Current Position: (1.234, 2.345, 0.000)
# 🔄 Current Orientation (roll, pitch, yaw): (0.000, 0.000, 1.570)
# ⏰ Pose data age: 0.123 seconds
# ✅ Pose data is within acceptable age range
```

### 示例3：使用实时位姿添加节点

```bash
# 添加节点时选择实时位姿模式
Enter command: N
Enter node name/ID (default: 3): 3
Use realtime pose data (y/n, default: y): y

# 系统会：
# 1. 检查位姿数据是否新鲜
# 2. 如果数据过期，等待新数据
# 3. 使用最新位姿创建节点
```

## 故障排除

### 常见问题

1. **位姿数据过期**
   - 使用 `RT` 命令检查位姿状态
   - 确保传感器正常工作
   - 调整 `pose_timeout` 参数

2. **无法获取实时位姿**
   - 检查 odometry 话题是否正常发布
   - 确认网络连接稳定
   - 使用 `wait_for_fresh_pose()` 等待新数据

3. **导航路径不准确**
   - 检查节点位置是否正确
   - 验证边的连接关系
   - 调整导航速度参数

### 调试命令

- `RT` - 检查实时位姿状态
- `G` - 获取当前位姿信息
- `I` - 获取点云信息
- `VH` - 检查可视化状态

## 技术特性

### 线程安全
- 使用锁机制确保多线程安全
- 后台线程处理ROS消息
- 主线程处理用户输入

### 实时性
- 支持实时位姿获取
- 可配置的超时机制
- 非阻塞式消息处理

### 可视化
- 实时点云可视化
- 轨迹显示
- 交互式3D界面

## 扩展功能

### 自定义配置
- 可调整位姿超时时间
- 可配置点云处理参数
- 支持自定义话题名称

### 数据导出
- 支持保存点云数据
- 可导出导航路径
- 支持多种文件格式

这个导航系统为机器人狗提供了完整的节点式导航解决方案，支持从建图到导航的完整工作流程，同时解决了实时位姿获取的时延问题。

## HTTP API 接口

### 启动HTTP服务器

```bash
# 启动HTTP Navigator Agent
python3 http_navigator_agent.py

# 自定义配置启动
python3 http_navigator_agent.py --host 0.0.0.0 --port 8080 --debug
```

### API端点

#### 健康检查和帮助
- `GET /api/health` - 健康检查
- `GET /api/help` - 获取完整API文档

#### 建图操作
- `POST /api/mapping/start` - 开始建图
- `POST /api/mapping/end` - 结束建图

#### 导航控制
- `POST /api/navigation/start` - 开始导航
- `POST /api/navigation/pause` - 暂停导航
- `POST /api/navigation/recover` - 恢复导航

#### 节点管理
- `POST /api/nodes/add` - 添加节点
- `POST /api/nodes/add_current` - 在当前位置添加节点
- `DELETE /api/nodes/delete` - 删除节点
- `POST /api/nodes/query` - 查询节点（新增）

#### 边管理
- `POST /api/edges/add` - 添加边
- `DELETE /api/edges/delete` - 删除边
- `POST /api/edges/query` - 查询边（新增）

#### 位姿操作
- `POST /api/pose/init` - 初始化位姿
- `POST /api/pose/relocation` - 开始重定位
- `GET /api/pose/current` - 获取当前位姿
- `GET /api/pose/realtime` - 获取实时位姿

#### 相机数据（新增）
- `POST /api/camera/data` - 获取相机数据

#### 机器人状态（新增）
- `GET /api/robot/nav_state` - 获取机器人导航状态

#### 可视化控制
- `POST /api/visualization/start` - 开始可视化
- `POST /api/visualization/stop` - 停止可视化
- `GET /api/visualization/status` - 获取可视化状态

#### 点云操作
- `POST /api/pointcloud/clear/environment` - 清除环境点云
- `POST /api/pointcloud/clear/trajectory` - 清除轨迹点云
- `POST /api/pointcloud/clear/all` - 清除所有点云
- `POST /api/pointcloud/save/environment` - 保存环境点云
- `POST /api/pointcloud/save/trajectory` - 保存轨迹点云
- `POST /api/pointcloud/save/combined` - 保存组合点云

#### 系统状态
- `GET /api/status` - 获取系统状态

### API使用示例

#### 查询节点
```bash
curl -X POST http://localhost:8080/api/nodes/query \
  -H 'Content-Type: application/json' \
  -d '{"seq":"index:123;","attribute":1}'
```

#### 获取相机数据
```bash
curl -X POST http://localhost:8080/api/camera/data \
  -H 'Content-Type: application/json' \
  -d '{"camera_names":["front","back"],"camera_modes":["RGB","RGB"]}'
```

#### 获取机器人状态
```bash
curl -X GET http://localhost:8080/api/robot/nav_state
```

#### 添加节点
```bash
curl -X POST http://localhost:8080/api/nodes/add \
  -H 'Content-Type: application/json' \
  -d '{"node_name":1,"x":1.0,"y":2.0,"z":0.0,"yaw":1.57}'
```

#### 开始建图
```bash
curl -X POST http://localhost:8080/api/mapping/start \
  -H 'Content-Type: application/json' \
  -d '{"seq":"index:123;","attribute":0}'
```

### API响应格式

所有API响应都采用统一的JSON格式：

```json
{
  "success": true/false,
  "message": "操作结果描述",
  "data": {...},  // 具体数据（可选）
  "timestamp": 1234567890.123
}
```

### 日志功能

HTTP Navigator Agent包含完整的日志记录功能：

- **日志文件位置**: `logs/http_navigator_agent_YYYYMMDD_HHMMSS.log`
- **日志级别**: INFO, WARNING, ERROR
- **日志内容**: 
  - API请求/响应记录
  - ROS2消息处理状态
  - 错误和异常信息
  - 系统启动/关闭事件

#### 查看日志
```bash
# 查看最新日志
tail -f logs/http_navigator_agent_*.log

# 查看特定时间的日志
ls logs/
cat logs/http_navigator_agent_20241204_143052.log
```

## 新增功能说明

### 1. 查询功能
- **节点查询**: 通过`query_node()`方法查询当前系统中的所有节点信息
- **边查询**: 通过`query_edge()`方法查询当前系统中的所有边信息
- **结果反馈**: 查询结果通过`/query_result_node`和`/query_result_edge`话题发布
- **确认机制**: 集成`wait_for_command_confirmation()`确保命令执行成功

### 2. 相机数据获取
- **多相机支持**: 支持前置、后置等多个相机同时获取
- **多模式支持**: RGB、DEPTH、WIDE等不同模式
- **Base64编码**: 图像数据采用Base64编码传输
- **状态监控**: 实时监控相机连接状态

### 3. 机器人状态监控
- **位置信息**: 实时位置坐标(x, y)和朝向角度(yaw, roll)
- **速度信息**: 线速度(vx, vy, v_linear)和角速度(vyaw)
- **电池信息**: 电池电量、充电次数、电池温度
- **传感器信息**: IMU温度、NTC温度传感器数据
- **系统状态**: 整体系统运行状态

### 4. HTTP接口增强
- **统一API格式**: 所有接口采用统一的请求/响应格式
- **完整错误处理**: 包含详细的错误信息和状态码
- **实时日志记录**: 记录所有API调用和系统事件
- **API文档**: 内置完整的API帮助文档

## 部署和使用

### 快速开始

1. **启动基础导航系统**
```bash
python3 navigator_example.py
```

2. **启动HTTP API服务**
```bash
python3 http_navigator_agent.py --host 0.0.0.0 --port 8080
```

3. **访问API文档**
```
http://YOUR_IP:8080/api/help
```

4. **健康检查**
```bash
curl http://YOUR_IP:8080/api/health
```

### 系统集成

该系统可以轻松集成到更大的机器人控制系统中：

- **ROS2兼容**: 完全兼容ROS2生态系统
- **网络接口**: 通过HTTP API支持远程控制
- **模块化设计**: 各功能模块独立，便于定制
- **扩展性**: 支持添加新的传感器和功能模块
