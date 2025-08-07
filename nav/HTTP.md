# 机器人狗导航系统 HTTP API 文档

## 📋 文档接口概览

### 🎯 核心功能接口

| 功能模块 | 主要接口 | 描述 |
|---------|---------|------|
| **自主建图** | `POST /robotic_control/navigation/autonomous_mapping` | 控制自主建图过程，支持超时和自动保存 |
| **定点导航** | `POST /robotic_control/navigation/fixed_point_nav` | 执行定点导航到指定坐标点 |
| **一键返回** | `POST /robotic_control/navigation/go_home` | 执行一键返回操作 |
| **设置导航参数** | `POST /robotic_control/navigation/set_auto_nav` | 设置自动导航参数 |
| **获取导航状态** | `GET /robotic_control/navigation/get_nav_state` | 获取机器人导航状态 |

### 🔧 接口参数说明

#### 自主建图参数
```json
{
  "command": 1,           // 1表示开始,0表示终止
  "save": true,           // 是否保存所建地图
  "save_path": "/path/to/map.pcd",  // 保存地图的路径（可选）
  "max_time_out": 3600    // 设置最长自主建图时间（秒）
}
```

#### 定点导航参数
```json
{
  "goal_coordinates": {    // 目标点坐标 {x, y, yaw}
    "x": 1.5,
    "y": 2.3,
    "yaw": 0.0
  },
  "goal_node_id": 5,      // 目标节点ID（可选）
  "map": "default"        // 使用的地图名称
}
```

#### 自动导航参数
```json
{
  "map": "default",                    // 使用的导航地图
  "area": [0.0, 0.0, 10.0, 10.0],    // 划定的导航区域范围
  "path_point": [1.0, 1.0, 5.0, 5.0, 9.0, 9.0]  // 设置的巡逻路径点
}
```

### 📊 完整接口列表

| 分类 | 接口 | 方法 | 描述 |
|------|------|------|------|
| **系统状态** | `/api/health` | GET | 健康检查 |
| | `/api/status` | GET | 系统状态 |
| | `/api/help` | GET | API帮助 |
| **建图操作** | `/api/mapping/start` | POST | 开始建图 |
| | `/api/mapping/end` | POST | 结束建图 |
| | `/robotic_control/navigation/autonomous_mapping` | POST | 自主建图 |
| **导航控制** | `/api/navigation/start_loop` | POST | 开始循环导航 |
| | `/api/navigation/pause` | POST | 暂停导航 |
| | `/api/navigation/recover` | POST | 恢复导航 |
| | `/robotic_control/navigation/fixed_point_nav` | POST | 定点导航 |
| | `/robotic_control/navigation/go_home` | POST | 一键返回 |
| | `/robotic_control/navigation/set_auto_nav` | POST | 设置自动导航 |
| | `/robotic_control/navigation/get_nav_state` | GET | 获取导航状态 |
| **节点管理** | `/api/nodes/add` | POST | 添加节点 |
| | `/api/nodes/add_current` | POST | 当前位置添加节点 |
| | `/api/nodes/delete` | DELETE | 删除节点 |
| **边管理** | `/api/edges/add` | POST | 添加边 |
| | `/api/edges/delete` | DELETE | 删除边 |
| **位姿操作** | `/api/pose/init` | POST | 初始化位姿 |
| | `/api/pose/relocation` | POST | 开始重定位 |
| | `/api/pose/current` | GET | 获取当前位姿 |
| | `/api/pose/realtime` | GET | 获取实时位姿 |
| **可视化控制** | `/api/visualization/start` | POST | 开始可视化 |
| | `/api/visualization/stop` | POST | 停止可视化 |
| | `/api/visualization/status` | GET | 可视化状态 |
| **点云操作** | `/api/pointcloud/clear/environment` | POST | 清除环境点云 |
| | `/api/pointcloud/clear/trajectory` | POST | 清除轨迹点云 |
| | `/api/pointcloud/clear/all` | POST | 清除所有点云 |
| | `/api/pointcloud/save/environment` | POST | 保存环境点云 |
| | `/api/pointcloud/save/trajectory` | POST | 保存轨迹点云 |
| | `/api/pointcloud/save/combined` | POST | 保存组合点云 |
| **相机数据** | `/api/camera/data` | POST | 获取相机数据 |

### 🚀 快速使用示例

#### 1. 自主建图
```bash
curl -X POST http://localhost:8080/robotic_control/navigation/autonomous_mapping \
  -H "Content-Type: application/json" \
  -d '{
    "command": 1,
    "save": true,
    "save_path": "/home/robot/maps/autonomous_map.pcd",
    "max_time_out": 1800
  }'
```

#### 2. 定点导航
```bash
curl -X POST http://localhost:8080/robotic_control/navigation/fixed_point_nav \
  -H "Content-Type: application/json" \
  -d '{
    "goal_coordinates": {
      "x": 5.0,
      "y": 3.0,
      "yaw": 1.57
    },
    "map": "default"
  }'
```

#### 3. 一键返回
```bash
curl -X POST http://localhost:8080/robotic_control/navigation/go_home \
  -H "Content-Type: application/json"
```

#### 4. 设置自动导航
```bash
curl -X POST http://localhost:8080/robotic_control/navigation/set_auto_nav \
  -H "Content-Type: application/json" \
  -d '{
    "map": "default",
    "area": [0.0, 0.0, 10.0, 10.0],
    "path_point": [1.0, 1.0, 5.0, 5.0, 9.0, 9.0]
  }'
```

#### 5. 获取导航状态
```bash
curl -X GET http://localhost:8080/robotic_control/navigation/get_nav_state
```

---

## 概述

本文档描述了机器人狗导航系统的HTTP REST API接口。该API提供了完整的导航控制功能，包括建图、导航、节点管理、位姿操作、可视化控制等。

**基础URL**: `http://localhost:8080`  
**默认端口**: 8080  
**内容类型**: `application/json`

## 快速开始

### 启动服务器

```bash
cd /path/to/nav
python http_navigator_agent.py --host 0.0.0.0 --port 8080
```

### 健康检查

```bash
curl http://localhost:8080/api/health
```

### 获取API帮助

```bash
curl http://localhost:8080/api/help
```

## API 端点分类

### 1. 系统状态

#### 1.1 健康检查
- **端点**: `GET /api/health`
- **描述**: 检查服务器运行状态
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "message": "HTTP Navigator Agent 运行正常",
  "timestamp": 1640995200.0,
  "ros_node": "navigator"
}
```

#### 1.2 系统状态
- **端点**: `GET /api/status`
- **描述**: 获取系统详细状态信息
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "status": {
    "navigator_running": true,
    "current_pose_available": true,
    "realtime_pose_available": true,
    "visualization_running": false,
    "environment_cloud_points": 15000,
    "trajectory_cloud_points": 5000,
    "total_cloud_points": 20000,
    "timestamp": 1640995200.0,
    "mapping_timeout_active": false
  },
  "message": "系统状态获取成功"
}
```

### 2. 建图操作

#### 2.1 开始建图
- **端点**: `POST /api/mapping/start`
- **描述**: 开始自主建图过程
- **参数**:
  ```json
  {
    "seq": "index:123;",
    "attribute": 0
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "建图命令已发送",
  "seq": "index:123;"
}
```

#### 2.2 结束建图
- **端点**: `POST /api/mapping/end`
- **描述**: 结束建图并保存地图
- **参数**:
  ```json
  {
    "seq": "index:123;",
    "floor_index": 0,
    "pcdmap_index": 0
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "结束建图命令已发送",
  "seq": "index:123;"
}
```

#### 2.3 自主建图
- **端点**: `POST /robotic_control/navigation/autonomous_mapping`
- **描述**: 控制自主建图过程，支持超时和自动保存
- **参数**:
  ```json
  {
    "command": 1,
    "save": true,
    "save_path": "/path/to/map.pcd",
    "max_time_out": 3600
  }
  ```
- **参数说明**:
  - `command`: 1表示开始，0表示终止
  - `save`: 是否保存所建地图
  - `save_path`: 保存地图的路径（可选）
  - `max_time_out`: 设置最长自主建图时间（秒）
- **响应示例**:
```json
{
  "success": true,
  "message": "自主建图已开始，将在3600秒后自动停止",
  "parameters": {
    "command": 1,
    "save": true,
    "save_path": "/path/to/map.pcd",
    "max_time_out": 3600
  }
}
```

### 3. 导航控制

#### 3.1 开始循环导航
- **端点**: `POST /api/navigation/start_loop`
- **描述**: 开始循环导航（包含重定位、导航启动、位姿初始化）
- **参数**:
  ```json
  {
    "seq": "index:123;"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "导航命令已发送",
  "seq": "index:123;"
}
```

#### 3.2 暂停导航
- **端点**: `POST /api/navigation/pause`
- **描述**: 暂停当前导航
- **参数**:
  ```json
  {
    "seq": "index:123;"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "暂停导航命令已发送",
  "seq": "index:123;"
}
```

#### 3.3 恢复导航
- **端点**: `POST /api/navigation/recover`
- **描述**: 恢复暂停的导航
- **参数**:
  ```json
  {
    "seq": "index:123;"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "恢复导航命令已发送",
  "seq": "index:123;"
}
```

#### 3.4 定点导航
- **端点**: `POST /robotic_control/navigation/fixed_point_nav`
- **描述**: 执行定点导航到指定坐标点
- **参数**:
  ```json
  {
    "goal_coordinates": {
      "x": 1.5,
      "y": 2.3,
      "yaw": 0.0
    },
    "goal_node_id": 5,
    "map": "default",
    "goal_yaw": 0.0
  }
  ```
- **参数说明**:
  - `goal_coordinates`: 目标点坐标对象 `{x, y, yaw}`
  - `goal_node_id`: 目标节点ID（可选）
  - `map`: 使用的地图名称
  - `goal_yaw`: 目标位姿俯仰角
- **响应示例**:
```json
{
  "success": true,
  "message": "定点导航已启动，目标点: (1.50, 2.30), 目标点ID: 5, 目标角度: 0.00°",
  "parameters": {
    "goal_coordinates": {"x": 1.5, "y": 2.3, "yaw": 0.0},
    "map": "default",
    "goal_yaw": 0.0,
    "current_pose": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "yaw": 0.0
    }
  }
}
```

#### 3.5 一键返回
- **端点**: `POST /robotic_control/navigation/go_home`
- **描述**: 执行一键返回操作
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "message": "一键返回已启动"
}
```

#### 3.6 设置自动导航
- **端点**: `POST /robotic_control/navigation/set_auto_nav`
- **描述**: 设置自动导航参数
- **参数**:
  ```json
  {
    "map": "default",
    "area": [0.0, 0.0, 10.0, 10.0],
    "path_point": [1.0, 1.0, 5.0, 5.0, 9.0, 9.0]
  }
  ```
- **参数说明**:
  - `map`: 使用的导航地图
  - `area`: 划定的导航区域范围 `[x1, y1, x2, y2]`
  - `path_point`: 设置的巡逻路径点 `[x1, y1, x2, y2, ...]`
- **响应示例**:
```json
{
  "success": true,
  "message": "自动导航参数设置成功",
  "parameters": {
    "map": "default",
    "area": [0.0, 0.0, 10.0, 10.0],
    "path_point": [1.0, 1.0, 5.0, 5.0, 9.0, 9.0]
  }
}
```

#### 3.7 获取导航状态
- **端点**: `GET /robotic_control/navigation/get_nav_state`
- **描述**: 获取机器人导航状态
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "nav_state": {
    "navigation_active": true,
    "current_node": 3,
    "target_node": 5,
    "navigation_mode": "loop"
  },
  "message": "导航状态获取成功",
  "timestamp": 1640995200.0
}
```

### 4. 节点管理

#### 4.1 添加节点
- **端点**: `POST /api/nodes/add`
- **描述**: 在指定位置添加导航节点
- **参数**:
  ```json
  {
    "node_name": 1,
    "x": 1.5,
    "y": 2.3,
    "z": 0.0,
    "yaw": 1.57,
    "seq": "index:123;"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "节点 1 已添加到位置 (1.5, 2.3, 0.0)",
  "node": {
    "name": 1,
    "x": 1.5,
    "y": 2.3,
    "z": 0.0,
    "yaw": 1.57
  }
}
```

#### 4.2 当前位置添加节点
- **端点**: `POST /api/nodes/add_current`
- **描述**: 在机器人当前位置添加节点
- **参数**:
  ```json
  {
    "node_name": 1,
    "seq": "index:123;",
    "use_realtime": true
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "节点 1 已添加到当前位置",
  "node_name": 1
}
```

#### 4.3 删除节点
- **端点**: `DELETE /api/nodes/delete`
- **描述**: 删除指定的导航节点
- **参数**:
  ```json
  {
    "node_ids": [1, 2, 3],
    "seq": "index:123;"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "节点 [1, 2, 3] 删除命令已发送",
  "deleted_nodes": [1, 2, 3]
}
```

### 5. 边管理

#### 5.1 添加边
- **端点**: `POST /api/edges/add`
- **描述**: 添加连接两个节点的边
- **参数**:
  ```json
  {
    "edge_name": 1,
    "start_node": 1,
    "end_node": 2,
    "dog_speed": 1.0,
    "seq": "index:123;"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "边 1 已添加 (节点1 -> 节点2)",
  "edge": {
    "name": 1,
    "start": 1,
    "end": 2,
    "speed": 1.0
  }
}
```

#### 5.2 删除边
- **端点**: `DELETE /api/edges/delete`
- **描述**: 删除指定的边
- **参数**:
  ```json
  {
    "edge_ids": [1, 2, 3],
    "seq": "index:123;"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "边 [1, 2, 3] 删除命令已发送",
  "deleted_edges": [1, 2, 3]
}
```

### 6. 位姿操作

#### 6.1 初始化位姿
- **端点**: `POST /api/pose/init`
- **描述**: 初始化机器人位姿
- **参数**:
  ```json
  {
    "seq": "index:123;",
    "translation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    },
    "quaternion": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "位姿初始化命令已发送",
  "translation": [0.0, 0.0, 0.0],
  "quaternion": [0.0, 0.0, 0.0, 1.0]
}
```

#### 6.2 开始重定位
- **端点**: `POST /api/pose/relocation`
- **描述**: 开始重定位过程
- **参数**:
  ```json
  {
    "seq": "index:123;",
    "attribute": 0
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "重定位命令已发送",
  "seq": "index:123;"
}
```

#### 6.3 获取当前位姿
- **端点**: `GET /api/pose/current`
- **描述**: 获取机器人当前位姿
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "pose": {
    "position": [1.234, 2.345, 0.0],
    "euler": [0.0, 0.0, 1.57],
    "quaternion": [0.0, 0.0, 0.707, 0.707]
  },
  "message": "位姿获取成功"
}
```

#### 6.4 获取实时位姿
- **端点**: `GET /api/pose/realtime`
- **描述**: 获取机器人实时位姿（检查数据新鲜度）
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "pose": {
    "position": [1.234, 2.345, 0.0],
    "euler": [0.0, 0.0, 1.57],
    "quaternion": [0.0, 0.0, 0.707, 0.707],
    "timestamp": 1640995200.0
  },
  "message": "实时位姿获取成功"
}
```

### 7. 可视化控制

#### 7.1 开始可视化
- **端点**: `POST /api/visualization/start`
- **描述**: 开始3D可视化
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "message": "可视化已开始"
}
```

#### 7.2 停止可视化
- **端点**: `POST /api/visualization/stop`
- **描述**: 停止3D可视化
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "message": "可视化已停止"
}
```

#### 7.3 可视化状态
- **端点**: `GET /api/visualization/status`
- **描述**: 获取可视化运行状态
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "running": true,
  "message": "可视化正在运行"
}
```

### 8. 点云操作

#### 8.1 清除环境点云
- **端点**: `POST /api/pointcloud/clear/environment`
- **描述**: 清除环境点云数据
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "message": "环境点云已清除"
}
```

#### 8.2 清除轨迹点云
- **端点**: `POST /api/pointcloud/clear/trajectory`
- **描述**: 清除轨迹点云数据
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "message": "轨迹点云已清除"
}
```

#### 8.3 清除所有点云
- **端点**: `POST /api/pointcloud/clear/all`
- **描述**: 清除所有点云数据
- **参数**: 无
- **响应示例**:
```json
{
  "success": true,
  "message": "所有点云已清除"
}
```

#### 8.4 保存环境点云
- **端点**: `POST /api/pointcloud/save/environment`
- **描述**: 保存环境点云到文件
- **参数**:
  ```json
  {
    "filename": "environment_cloud.pcd"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "环境点云已保存到: environment_cloud.pcd",
  "filename": "environment_cloud.pcd"
}
```

#### 8.5 保存轨迹点云
- **端点**: `POST /api/pointcloud/save/trajectory`
- **描述**: 保存轨迹点云到文件
- **参数**:
  ```json
  {
    "filename": "trajectory_cloud.pcd"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "轨迹点云已保存到: trajectory_cloud.pcd",
  "filename": "trajectory_cloud.pcd"
}
```

#### 8.6 保存组合点云
- **端点**: `POST /api/pointcloud/save/combined`
- **描述**: 保存环境+轨迹组合点云到文件
- **参数**:
  ```json
  {
    "filename": "combined_map.pcd"
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "message": "组合点云已保存到: combined_map.pcd",
  "filename": "combined_map.pcd"
}
```

### 9. 相机数据

#### 9.1 获取相机数据
- **端点**: `POST /api/camera/data`
- **描述**: 获取指定相机的图像数据
- **参数**:
  ```json
  {
    "camera_names": ["front", "back"],
    "camera_modes": ["RGB", "RGB"]
  }
  ```
- **响应示例**:
```json
{
  "success": true,
  "code": "000000",
  "message": "相机数据获取成功",
  "data": {
    "front": {
      "camera_id": 12345678,
      "name": "front",
      "mode": "RGB",
      "status": true,
      "frame_base64": "iVBORw0KGgoAAAANSUhEUgAA..."
    },
    "back": {
      "camera_id": 87654321,
      "name": "back",
      "mode": "RGB",
      "status": true,
      "frame_base64": "iVBORw0KGgoAAAANSUhEUgAA..."
    }
  },
  "camera_count": 2,
  "timestamp": 1640995200.0
}
```

## 错误处理

### 错误响应格式

所有API在发生错误时都会返回以下格式：

```json
{
  "success": false,
  "error": "错误描述信息"
}
```

### 常见HTTP状态码

- `200 OK`: 请求成功
- `400 Bad Request`: 请求参数错误
- `500 Internal Server Error`: 服务器内部错误

### 错误示例

```json
{
  "success": false,
  "error": "缺少必要参数: node_name, x, y"
}
```

## 使用示例

### 完整的导航工作流程

1. **开始建图**
```bash
curl -X POST http://localhost:8080/api/mapping/start \
  -H "Content-Type: application/json" \
  -d '{"seq": "index:123;", "attribute": 0}'
```

2. **添加导航节点**
```bash
curl -X POST http://localhost:8080/api/nodes/add \
  -H "Content-Type: application/json" \
  -d '{
    "node_name": 1,
    "x": 1.0,
    "y": 1.0,
    "z": 0.0,
    "yaw": 0.0
  }'
```

3. **添加导航边**
```bash
curl -X POST http://localhost:8080/api/edges/add \
  -H "Content-Type: application/json" \
  -d '{
    "edge_name": 1,
    "start_node": 1,
    "end_node": 2,
    "dog_speed": 1.0
  }'
```

4. **结束建图**
```bash
curl -X POST http://localhost:8080/api/mapping/end \
  -H "Content-Type: application/json" \
  -d '{"seq": "index:124;", "floor_index": 0, "pcdmap_index": 0}'
```

5. **开始导航**
```bash
curl -X POST http://localhost:8080/api/navigation/start_loop \
  -H "Content-Type: application/json" \
  -d '{"seq": "index:125;"}'
```

### 定点导航示例

```bash
curl -X POST http://localhost:8080/robotic_control/navigation/fixed_point_nav \
  -H "Content-Type: application/json" \
  -d '{
    "goal_coordinates": {
      "x": 5.0,
      "y": 3.0,
      "yaw": 1.57
    },
    "map": "default"
  }'
```

### 自主建图示例

```bash
curl -X POST http://localhost:8080/robotic_control/navigation/autonomous_mapping \
  -H "Content-Type: application/json" \
  -d '{
    "command": 1,
    "save": true,
    "save_path": "/home/robot/maps/autonomous_map.pcd",
    "max_time_out": 1800
  }'
```

## 注意事项

1. **序列号管理**: 每个命令都应该使用唯一的序列号，格式为 `"index:数字;"`
2. **位姿数据**: 实时位姿数据有时延，建议在关键操作前检查位姿数据的新鲜度
3. **点云大小**: 大量点云数据可能影响性能，建议定期清理不需要的点云
4. **网络连接**: 确保客户端与服务器在同一网络段，且防火墙允许相应端口
5. **日志记录**: 服务器会自动记录所有API调用到日志文件中

## 开发环境

### 依赖项

- Python 3.8+
- ROS2 Humble
- Flask
- rclpy
- numpy
- open3d
- opencv-python

### 启动命令

```bash
# 基本启动
python http_navigator_agent.py

# 指定主机和端口
python http_navigator_agent.py --host 0.0.0.0 --port 8080

# 调试模式
python http_navigator_agent.py --debug
```

### 配置文件

服务器会自动创建日志文件在 `logs/` 目录下，格式为：
`http_navigator_agent_YYYYMMDD_HHMMSS.log`

## 更新日志

### v1.0.0
- 初始版本发布
- 支持基本的建图、导航、节点管理功能
- 添加HTTP REST API接口
- 支持相机数据获取
- 支持点云操作和可视化

### v1.1.0
- 添加自主建图功能
- 添加定点导航功能
- 添加一键返回功能
- 添加自动导航参数设置
- 改进错误处理和日志记录 
