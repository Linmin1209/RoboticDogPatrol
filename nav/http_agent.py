#!/usr/bin/env python3
"""
HTTP Navigator Agent - 机器人狗导航系统HTTP服务器

提供HTTP REST API接口，接收网络请求并调用Navigator类功能
可以在同网段下的其他终端向此服务器发送请求进行命令下发
"""

from flask import Flask, request, jsonify
import threading
import time
import json
import logging
from typing import Dict, Any, Optional
import rclpy
from navigator import Navigator


class HTTPNavigatorAgent:
    """
    Navigator的HTTP服务器代理
    
    启动HTTP服务器，提供REST API接口来远程控制Navigator功能
    """
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8080, debug: bool = False):
        """
        初始化HTTP Navigator Agent
        
        Args:
            host: 服务器监听地址 (0.0.0.0表示监听所有网络接口)
            port: 服务器监听端口
            debug: 是否开启调试模式
        """
        self.host = host
        self.port = port
        self.debug = debug
        
        # 初始化ROS2和Navigator
        rclpy.init()
        self.navigator = Navigator(enable_visualization=True)
        
        # 创建Flask应用
        self.app = Flask(__name__)
        self.app.config['JSON_AS_ASCII'] = False  # 支持中文JSON
        
        # 设置日志
        if not debug:
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.WARNING)
        
        # 注册路由
        self._register_routes()
        
        # ROS2 spin线程
        self.ros_thread = None
        self.running = False
        
        print(f"🌐 HTTP Navigator Agent 初始化完成")
        print(f"📡 服务器地址: http://{host}:{port}")
        print(f"📖 API文档: http://{host}:{port}/api/help")
    
    def _register_routes(self):
        """注册所有API路由"""
        
        # 健康检查和帮助
        @self.app.route('/api/health', methods=['GET'])
        def health_check():
            """健康检查"""
            return jsonify({
                "success": True,
                "message": "HTTP Navigator Agent 运行正常",
                "timestamp": time.time(),
                "ros_node": "navigator"
            })
        
        @self.app.route('/api/help', methods=['GET'])
        def api_help():
            """API帮助文档"""
            help_doc = {
                "name": "HTTP Navigator Agent API",
                "version": "1.0",
                "description": "机器人狗导航系统HTTP API接口",
                "endpoints": {
                    "健康检查": "GET /api/health",
                    "建图操作": {
                        "开始建图": "POST /api/mapping/start",
                        "结束建图": "POST /api/mapping/end"
                    },
                    "导航控制": {
                        "开始导航": "POST /api/navigation/start",
                        "暂停导航": "POST /api/navigation/pause", 
                        "恢复导航": "POST /api/navigation/recover"
                    },
                    "节点管理": {
                        "添加节点": "POST /api/nodes/add",
                        "当前位置添加节点": "POST /api/nodes/add_current",
                        "删除节点": "DELETE /api/nodes/delete"
                    },
                    "边管理": {
                        "添加边": "POST /api/edges/add",
                        "删除边": "DELETE /api/edges/delete"
                    },
                    "位姿操作": {
                        "初始化位姿": "POST /api/pose/init",
                        "开始重定位": "POST /api/pose/relocation",
                        "获取当前位姿": "GET /api/pose/current",
                        "获取实时位姿": "GET /api/pose/realtime"
                    },
                    "可视化": {
                        "开始可视化": "POST /api/visualization/start",
                        "停止可视化": "POST /api/visualization/stop",
                        "可视化状态": "GET /api/visualization/status"
                    },
                    "点云操作": {
                        "清除环境点云": "POST /api/pointcloud/clear/environment",
                        "清除轨迹点云": "POST /api/pointcloud/clear/trajectory",
                        "清除所有点云": "POST /api/pointcloud/clear/all",
                        "保存环境点云": "POST /api/pointcloud/save/environment",
                        "保存轨迹点云": "POST /api/pointcloud/save/trajectory",
                        "保存组合点云": "POST /api/pointcloud/save/combined"
                    },
                    "系统状态": "GET /api/status"
                },
                "usage_example": {
                    "curl": "curl -X POST http://YOUR_IP:8080/api/mapping/start -H 'Content-Type: application/json' -d '{\"seq\":\"index:123;\",\"attribute\":0}'"
                }
            }
            return jsonify(help_doc)
        
        # 建图操作
        @self.app.route('/api/mapping/start', methods=['POST'])
        def start_mapping():
            """开始建图"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                attribute = data.get('attribute', 0)
                
                result = self.navigator.start_mapping(seq, attribute)
                return jsonify({
                    "success": result,
                    "message": "建图命令已发送" if result else "建图命令发送失败",
                    "seq": seq
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/mapping/end', methods=['POST'])
        def end_mapping():
            """结束建图"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                floor_index = data.get('floor_index', 0)
                pcdmap_index = data.get('pcdmap_index', 0)
                
                result = self.navigator.end_mapping(seq, floor_index, pcdmap_index)
                return jsonify({
                    "success": result,
                    "message": "结束建图命令已发送" if result else "结束建图命令发送失败",
                    "seq": seq
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 导航控制
        @self.app.route('/api/navigation/start', methods=['POST'])
        def start_navigation():
            """开始导航"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                
                result = self.navigator.start_navigation(seq)
                return jsonify({
                    "success": result,
                    "message": "导航命令已发送" if result else "导航命令发送失败",
                    "seq": seq
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/navigation/pause', methods=['POST'])
        def pause_navigation():
            """暂停导航"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                
                result = self.navigator.pause_navigation(seq)
                return jsonify({
                    "success": result,
                    "message": "暂停导航命令已发送" if result else "暂停导航命令发送失败",
                    "seq": seq
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/navigation/recover', methods=['POST'])
        def recover_navigation():
            """恢复导航"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                
                result = self.navigator.recover_navigation(seq)
                return jsonify({
                    "success": result,
                    "message": "恢复导航命令已发送" if result else "恢复导航命令发送失败",
                    "seq": seq
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 节点管理
        @self.app.route('/api/nodes/add', methods=['POST'])
        def add_node():
            """添加节点"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({"success": False, "error": "缺少请求数据"}), 400
                
                node_name = data.get('node_name')
                x = data.get('x')
                y = data.get('y')
                z = data.get('z', 0.0)
                yaw = data.get('yaw', 1.57)
                seq = data.get('seq', 'index:123;')
                
                if node_name is None or x is None or y is None:
                    return jsonify({"success": False, "error": "缺少必要参数: node_name, x, y"}), 400
                
                self.navigator.add_node(node_name, x, y, z, yaw, seq)
                return jsonify({
                    "success": True,
                    "message": f"节点 {node_name} 已添加到位置 ({x}, {y}, {z})",
                    "node": {"name": node_name, "x": x, "y": y, "z": z, "yaw": yaw}
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/nodes/add_current', methods=['POST'])
        def add_node_at_current_pose():
            """在当前位置添加节点"""
            try:
                data = request.get_json() or {}
                node_name = data.get('node_name')
                seq = data.get('seq', 'index:123;')
                use_realtime = data.get('use_realtime', True)
                
                if node_name is None:
                    return jsonify({"success": False, "error": "缺少必要参数: node_name"}), 400
                
                result = self.navigator.add_node_at_current_pose(node_name, seq, use_realtime)
                return jsonify({
                    "success": result,
                    "message": f"节点 {node_name} 已添加到当前位置" if result else "添加节点失败",
                    "node_name": node_name
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/nodes/delete', methods=['DELETE'])
        def delete_nodes():
            """删除节点"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({"success": False, "error": "缺少请求数据"}), 400
                
                node_ids = data.get('node_ids')
                seq = data.get('seq', 'index:123;')
                
                if not node_ids:
                    return jsonify({"success": False, "error": "缺少必要参数: node_ids"}), 400
                
                self.navigator.delete_node(node_ids, seq)
                return jsonify({
                    "success": True,
                    "message": f"节点 {node_ids} 删除命令已发送",
                    "deleted_nodes": node_ids
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 边管理
        @self.app.route('/api/edges/add', methods=['POST'])
        def add_edge():
            """添加边"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({"success": False, "error": "缺少请求数据"}), 400
                
                edge_name = data.get('edge_name')
                start_node = data.get('start_node')
                end_node = data.get('end_node')
                dog_speed = data.get('dog_speed', 1.0)
                seq = data.get('seq', 'index:123;')
                
                if edge_name is None or start_node is None or end_node is None:
                    return jsonify({"success": False, "error": "缺少必要参数: edge_name, start_node, end_node"}), 400
                
                self.navigator.add_edge(edge_name, start_node, end_node, dog_speed, seq)
                return jsonify({
                    "success": True,
                    "message": f"边 {edge_name} 已添加 (节点{start_node} -> 节点{end_node})",
                    "edge": {"name": edge_name, "start": start_node, "end": end_node, "speed": dog_speed}
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/edges/delete', methods=['DELETE'])
        def delete_edges():
            """删除边"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({"success": False, "error": "缺少请求数据"}), 400
                
                edge_ids = data.get('edge_ids')
                seq = data.get('seq', 'index:123;')
                
                if not edge_ids:
                    return jsonify({"success": False, "error": "缺少必要参数: edge_ids"}), 400
                
                self.navigator.delete_edge(edge_ids, seq)
                return jsonify({
                    "success": True,
                    "message": f"边 {edge_ids} 删除命令已发送",
                    "deleted_edges": edge_ids
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 位姿操作
        @self.app.route('/api/pose/init', methods=['POST'])
        def init_pose():
            """初始化位姿"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                translation = data.get('translation', {})
                quaternion = data.get('quaternion', {})
                
                trans_tuple = (
                    translation.get('x', 0.0),
                    translation.get('y', 0.0), 
                    translation.get('z', 0.0)
                )
                quat_tuple = (
                    quaternion.get('x', 0.0),
                    quaternion.get('y', 0.0),
                    quaternion.get('z', 0.0),
                    quaternion.get('w', 1.0)
                )
                
                result = self.navigator.pose_init(seq, trans_tuple, quat_tuple)
                return jsonify({
                    "success": result,
                    "message": "位姿初始化命令已发送" if result else "位姿初始化命令发送失败",
                    "translation": trans_tuple,
                    "quaternion": quat_tuple
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pose/relocation', methods=['POST'])
        def start_relocation():
            """开始重定位"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                attribute = data.get('attribute', 0)
                
                result = self.navigator.start_relocation(seq, attribute)
                return jsonify({
                    "success": result,
                    "message": "重定位命令已发送" if result else "重定位命令发送失败",
                    "seq": seq
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pose/current', methods=['GET'])
        def get_current_pose():
            """获取当前位姿"""
            try:
                pose = self.navigator.get_current_pose()
                if pose:
                    return jsonify({
                        "success": True,
                        "pose": pose,
                        "message": "位姿获取成功"
                    })
                else:
                    return jsonify({
                        "success": False,
                        "message": "暂无位姿数据"
                    })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pose/realtime', methods=['GET'])
        def get_realtime_pose():
            """获取实时位姿"""
            try:
                pose = self.navigator.get_realtime_pose()
                if pose:
                    return jsonify({
                        "success": True,
                        "pose": pose,
                        "message": "实时位姿获取成功"
                    })
                else:
                    return jsonify({
                        "success": False,
                        "message": "实时位姿数据不可用或已过期"
                    })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 可视化控制
        @self.app.route('/api/visualization/start', methods=['POST'])
        def start_visualization():
            """开始可视化"""
            try:
                self.navigator.start_visualization()
                return jsonify({
                    "success": True,
                    "message": "可视化已开始"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/visualization/stop', methods=['POST'])
        def stop_visualization():
            """停止可视化"""
            try:
                self.navigator.stop_visualization()
                return jsonify({
                    "success": True,
                    "message": "可视化已停止"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/visualization/status', methods=['GET'])
        def visualization_status():
            """获取可视化状态"""
            try:
                running = self.navigator.is_visualization_running()
                return jsonify({
                    "success": True,
                    "running": running,
                    "message": "可视化正在运行" if running else "可视化已停止"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 点云操作
        @self.app.route('/api/pointcloud/clear/environment', methods=['POST'])
        def clear_environment_cloud():
            """清除环境点云"""
            try:
                self.navigator.clear_accumulated_cloud()
                return jsonify({
                    "success": True,
                    "message": "环境点云已清除"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pointcloud/clear/trajectory', methods=['POST'])
        def clear_trajectory_cloud():
            """清除轨迹点云"""
            try:
                self.navigator.clear_trajectory_cloud()
                return jsonify({
                    "success": True,
                    "message": "轨迹点云已清除"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pointcloud/clear/all', methods=['POST'])
        def clear_all_clouds():
            """清除所有点云"""
            try:
                self.navigator.clear_all_clouds()
                return jsonify({
                    "success": True,
                    "message": "所有点云已清除"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pointcloud/save/environment', methods=['POST'])
        def save_environment_cloud():
            """保存环境点云"""
            try:
                data = request.get_json() or {}
                filename = data.get('filename', 'accumulated_cloud.pcd')
                
                self.navigator.save_accumulated_cloud(filename)
                return jsonify({
                    "success": True,
                    "message": f"环境点云已保存到: {filename}",
                    "filename": filename
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pointcloud/save/trajectory', methods=['POST'])
        def save_trajectory_cloud():
            """保存轨迹点云"""
            try:
                data = request.get_json() or {}
                filename = data.get('filename', 'trajectory_cloud.pcd')
                
                self.navigator.save_trajectory_cloud(filename)
                return jsonify({
                    "success": True,
                    "message": f"轨迹点云已保存到: {filename}",
                    "filename": filename
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/pointcloud/save/combined', methods=['POST'])
        def save_combined_cloud():
            """保存组合点云"""
            try:
                data = request.get_json() or {}
                filename = data.get('filename', 'combined_map.pcd')
                
                self.navigator.save_combined_cloud(filename)
                return jsonify({
                    "success": True,
                    "message": f"组合点云已保存到: {filename}",
                    "filename": filename
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 系统状态
        @self.app.route('/api/status', methods=['GET'])
        def get_system_status():
            """获取系统状态"""
            try:
                current_pose = self.navigator.get_current_pose()
                realtime_pose = self.navigator.get_realtime_pose()
                viz_running = self.navigator.is_visualization_running()
                cloud_size = self.navigator.get_cloud_size()
                traj_size = self.navigator.get_trajectory_size()
                
                status = {
                    "navigator_running": True,
                    "current_pose_available": current_pose is not None,
                    "realtime_pose_available": realtime_pose is not None,
                    "visualization_running": viz_running,
                    "environment_cloud_points": cloud_size,
                    "trajectory_cloud_points": traj_size,
                    "total_cloud_points": cloud_size + traj_size,
                    "timestamp": time.time()
                }
                
                return jsonify({
                    "success": True,
                    "status": status,
                    "message": "系统状态获取成功"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
    
    def _start_ros_thread(self):
        """启动ROS2 spin线程"""
        self.running = True
        while self.running:
            try:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)
            except Exception as e:
                print(f"ROS spin error: {e}")
                break
    
    def start(self):
        """启动HTTP服务器"""
        try:
            # 启动ROS2线程
            self.ros_thread = threading.Thread(target=self._start_ros_thread, daemon=True)
            self.ros_thread.start()
            
            print(f"🚀 HTTP Navigator Agent 启动中...")
            print(f"📡 服务器地址: http://{self.host}:{self.port}")
            print(f"📖 API文档: http://{self.host}:{self.port}/api/help")
            print(f"🔍 健康检查: http://{self.host}:{self.port}/api/health")
            print("✅ 服务器已启动，可以接收命令!")
            
            # 启动Flask服务器
            self.app.run(host=self.host, port=self.port, debug=self.debug, threaded=True)
            
        except KeyboardInterrupt:
            print("\n⚠️ 收到中断信号，正在关闭服务器...")
            self.stop()
        except Exception as e:
            print(f"❌ 服务器启动失败: {e}")
            self.stop()
    
    def stop(self):
        """停止服务器"""
        self.running = False
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=2.0)
        
        if hasattr(self, 'navigator'):
            self.navigator.stop_visualization()
            self.navigator.destroy_node()
        
        rclpy.shutdown()
        print("🛑 HTTP Navigator Agent 已停止")


def main():
    """启动HTTP Navigator Agent"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HTTP Navigator Agent - 机器人狗导航系统HTTP服务器')
    parser.add_argument('--host', default='0.0.0.0', help='服务器监听地址 (默认: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8080, help='服务器监听端口 (默认: 8080)')
    parser.add_argument('--debug', action='store_true', help='开启调试模式')
    
    args = parser.parse_args()
    
    print("🎮 HTTP Navigator Agent")
    print("=" * 50)
    print("机器人狗导航系统HTTP服务器")
    print(f"监听地址: {args.host}:{args.port}")
    print("按 Ctrl+C 停止服务器")
    print("=" * 50)
    
    # 创建并启动agent
    agent = HTTPNavigatorAgent(host=args.host, port=args.port, debug=args.debug)
    agent.start()


if __name__ == "__main__":
    main()
