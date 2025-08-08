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
import os
from datetime import datetime
from typing import Dict, Any, Optional
import rclpy
from navigator import Navigator
from dataclasses import dataclass


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
        
        # 设置日志系统
        self._setup_logging()
        
        # 初始化ROS2和Navigator
        rclpy.init()
        self.navigator = Navigator(enable_visualization=True)
        
        # 创建Flask应用
        self.app = Flask(__name__)
        self.app.config['JSON_AS_ASCII'] = False  # 支持中文JSON
        
        # 设置Flask日志
        if not debug:
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.WARNING)
        
        # 注册路由
        self._register_routes()
        
        # ROS2 spin线程
        self.ros_thread = None
        self.running = False
        
        # 超时管理
        self.timeout_threads = {}  # 存储超时线程的字典
        self.mapping_timeout = None  # 当前建图超时时间
        
        self.logger.info(f"🌐 HTTP Navigator Agent 初始化完成")
        self.logger.info(f"📡 服务器地址: http://{host}:{port}")
        self.logger.info(f"📖 API文档: http://{host}:{port}/api/help")
        print(f"🌐 HTTP Navigator Agent 初始化完成")
        print(f"📡 服务器地址: http://{host}:{port}")
        print(f"📖 API文档: http://{host}:{port}/api/help")
    
    def _log_api_call(self, endpoint: str, method: str, data: dict = None, success: bool = True, message: str = ""):
        """记录API调用日志"""
        try:
            client_ip = request.remote_addr if request else "unknown"
            log_msg = f"API调用: {method} {endpoint} - 客户端IP: {client_ip}"
            
            if data:
                # 不记录敏感数据，只记录关键参数
                safe_data = {}
                for key, value in data.items():
                    if key in ['camera_names', 'seq', 'node_name', 'edge_name', 'x', 'y', 'z']:
                        safe_data[key] = value
                    elif 'password' not in key.lower() and 'token' not in key.lower():
                        safe_data[key] = str(value)[:100] if isinstance(value, str) else value
                log_msg += f" - 参数: {safe_data}"
            
            if success:
                self.logger.info(f"{log_msg} - 成功: {message}")
            else:
                self.logger.warning(f"{log_msg} - 失败: {message}")
                
        except Exception as e:
            self.logger.error(f"记录API日志时出错: {e}")
    
    def _setup_logging(self):
        """设置日志系统"""
        # 创建logs目录
        if not os.path.exists('logs'):
            os.makedirs('logs')
        
        # 生成日志文件名（包含时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"logs/http_navigator_agent_{timestamp}.log"
        
        # 配置日志格式
        log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        date_format = '%Y-%m-%d %H:%M:%S'
        
        # 创建logger
        self.logger = logging.getLogger('HTTPNavigatorAgent')
        self.logger.setLevel(logging.INFO)
        
        # 创建文件处理器
        file_handler = logging.FileHandler(log_filename, encoding='utf-8')
        file_handler.setLevel(logging.INFO)
        file_formatter = logging.Formatter(log_format, date_format)
        file_handler.setFormatter(file_formatter)
        
        # 创建控制台处理器（如果是debug模式）
        if self.debug:
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.INFO)
            console_formatter = logging.Formatter(log_format, date_format)
            console_handler.setFormatter(console_formatter)
            self.logger.addHandler(console_handler)
        
        # 添加处理器到logger
        self.logger.addHandler(file_handler)
        
        # 记录启动信息
        self.logger.info("=" * 50)
        self.logger.info("HTTP Navigator Agent 启动")
        self.logger.info(f"日志文件: {log_filename}")
        self.logger.info(f"调试模式: {self.debug}")
        self.logger.info("=" * 50)
    
    def _register_routes(self):
        """注册所有API路由"""
        
        # 健康检查和帮助
        @self.app.route('/api/health', methods=['GET'])
        def health_check():
            """健康检查"""
            self.logger.info("API调用: GET /api/health")
            response = {
                "success": True,
                "message": "HTTP Navigator Agent 运行正常",
                "timestamp": time.time(),
                "ros_node": "navigator"
            }
            self.logger.info(f"健康检查响应: {response['message']}")
            return jsonify(response)
        
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
                        "开始导航": "POST /api/navigation/start_loop",
                        "暂停导航": "POST /api/navigation/pause", 
                        "恢复导航": "POST /api/navigation/recover"

                    },
                    "节点管理": {
                        "添加节点": "POST /api/nodes/add",
                        "当前位置添加节点": "POST /api/nodes/add_current",
                        "删除节点": "DELETE /api/nodes/delete",
                        "删除所有节点": "DELETE /api/nodes/delete_all",
                        "关闭所有节点": "POST /api/nodes/close_all",
                        "查询节点": "POST /api/nodes/query"
                    },
                    "边管理": {
                        "添加边": "POST /api/edges/add",
                        "删除边": "DELETE /api/edges/delete",
                        "删除所有边": "DELETE /api/edges/delete_all",
                        "查询边": "POST /api/edges/query"
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
                    "网络分析": {
                        "网络概览": "GET /api/network/overview",
                        "路径分析": "POST /api/path/analysis"
                    },
                    "内部存储管理": {
                        "发布所有节点": "POST /api/internal/nodes/publish",
                        "发布所有边": "POST /api/internal/edges/publish",
                        "发布所有节点和边": "POST /api/internal/publish_all",
                        "清除内部存储": "POST /api/internal/clear",
                        "获取内部存储信息": "GET /api/internal/info"
                    },
                    "自动收集功能": {
                        "自动收集节点": "POST /api/auto_collect/node",
                        "收集并保存": "POST /api/auto_collect/save",
                        "自动收集循环": "POST /api/auto_collect/loop",
                        "准备收集": "POST /api/auto_collect/prepare",
                        "清除并开始映射": "POST /api/auto_collect/clear_and_map",
                        "获取收集状态": "GET /api/auto_collect/status"
                    },
                    "Notice 缓存控制": {
                        "配置缓存": "POST /api/notice_cache/configure",
                        "获取缓存状态": "GET /api/notice_cache/status",
                        "测试缓存": "POST /api/notice_cache/test"
                    },
                    "其他功能": {
                        "获取云信息": "GET /api/cloud/info",
                        "设置下采样参数": "POST /api/downsample/configure"
                    },
                    "系统状态": "GET /api/status",
                    "相机数据": {
                        "获取相机数据": "POST /api/camera/data"
                    },
                    "文档接口": {
                        "自主建图": {
                            "endpoint": "POST /robotic_control/navigation/autonomous_mapping",
                            "description": "控制自主建图过程",
                            "parameters": {
                                "command": {
                                    "type": "Int",
                                    "required": True,
                                    "description": "1表示开始,0表示终止"
                                },
                                "save": {
                                    "type": "Bool", 
                                    "required": True,
                                    "description": "是否保存所建地图"
                                },
                                "save_path": {
                                    "type": "String",
                                    "required": False,
                                    "description": "保存地图的路径"
                                },
                                "max_time_out": {
                                    "type": "Long",
                                    "required": False,
                                    "description": "设置最长自主建图时间（秒）"
                                }
                            }
                        },
                        "机器狗定点导航": {
                            "endpoint": "POST /robotic_control/navigation/fixed_point_nav",
                            "description": "执行定点导航到指定坐标点",
                            "parameters": {
                                "goal_coordinates": {
                                    "type": "Dict",
                                    "required": False,
                                    "description": "单个导航目标点的二维坐标值 {x, y, yaw}"
                                },
                                "goal_node_id": {
                                    "type": "Int",
                                    "required": False,
                                    "description": "单个导航目标点的ID"
                                },
                                "map": {
                                    "type": "String",
                                    "required": False,
                                    "description": "使用预先构建的地图"
                                },
                               
                            }
                        },
                        "机器狗一键返回": "POST /robotic_control/navigation/go_home",
                        "设置导航参数": {
                            "endpoint": "POST /robotic_control/navigation/set_auto_nav",
                            "description": "设置自动导航参数",
                            "parameters": {
                                "map": {
                                    "type": "String",
                                    "required": False,
                                    "description": "使用的导航地图"
                                },
                                "area": {
                                    "type": "List<Float>",
                                    "required": False,
                                    "description": "划定的导航区域范围"
                                },
                                "path_point": {
                                    "type": "List<Float>",
                                    "required": False,
                                    "description": "设置的巡逻路径点"
                                }
                            }
                        },
                        "获取导航状态": "GET /robotic_control/navigation/get_nav_state",

                    },
                    "机械臂控制": {
                        "获取机械臂位姿": {
                            "endpoint": "POST /robotic_control/robotic_arm/get_pose",
                            "description": "获取机械臂末端位姿",
                            "parameters": {},
                            "response": {
                                "code": "String",
                                "message": "String", 
                                "data": {
                                    "status": "Bool",
                                    "x": "Float",
                                    "y": "Float",
                                    "z": "Float",
                                    "roll": "Float",
                                    "yaw": "Float",
                                    "pitch": "Float"
                                }
                            }
                        },
                        "机械臂返回默认位置": {
                            "endpoint": "POST /robotic_control/robotic_arm/go_home",
                            "description": "机械臂返回默认位置",
                            "parameters": {},
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        },
                        "机械臂移动到指定位置": {
                            "endpoint": "POST /robotic_control/robotic_arm/move_to",
                            "description": "机械臂移动到指定位置",
                            "parameters": {
                                "x": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "x方向位置"
                                },
                                "y": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "y方向位置"
                                },
                                "z": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "z方向位置"
                                },
                                "roll": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角roll"
                                },
                                "yaw": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角yaw"
                                },
                                "pitch": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角pitch"
                                }
                            },
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        },
                        "机械臂末端移动到指定位置": {
                            "endpoint": "POST /robotic_control/robotic_arm/ee_move_to",
                            "description": "机械臂末端移动到指定位置，仅移动末端关节，用于微调位姿",
                            "parameters": {
                                "x": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "x方向位置"
                                },
                                "y": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "y方向位置"
                                },
                                "z": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "z方向位置"
                                },
                                "roll": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角roll"
                                },
                                "yaw": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角yaw"
                                },
                                "pitch": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角pitch"
                                }
                            },
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        },
                        "机械臂末端相机获取": {
                            "endpoint": "POST /robotic_control/robotic_arm/get_camera",
                            "description": "机械臂末端相机获取图像数据",
                            "parameters": {
                                "rgb": {
                                    "type": "Bool",
                                    "required": False,
                                    "description": "是否需要RGB图像",
                                    "default": False
                                },
                                "depth": {
                                    "type": "Bool",
                                    "required": False,
                                    "description": "是否需要深度图像",
                                    "default": False
                                }
                            },
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool",
                                    "images": "List"
                                }
                            }
                        },
                        "机械臂夹爪开合": {
                            "endpoint": "POST /robotic_control/robotic_arm/set_gripper",
                            "description": "机械臂夹爪开合控制",
                            "parameters": {
                                "value": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "开合角度，范围(0,1)"
                                }
                            },
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        },
                        "机械臂开关门": {
                            "endpoint": "POST /robotic_control/open_door",
                            "description": "机械臂开关门操作",
                            "parameters": {
                                "open": {
                                    "type": "Bool",
                                    "required": True,
                                    "description": "开门/关门"
                                },
                                "direction": {
                                    "type": "String",
                                    "required": True,
                                    "description": "开门方向，值为'left'或'right'"
                                }
                            },
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        }
                    },
                    "协作臂控制": {
                        "协作臂返回默认位置": {
                            "endpoint": "POST /robotic_control/collaborative_arm/go_home",
                            "description": "协作臂返回默认位置",
                            "parameters": {},
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        },
                        "协作臂移动到指定位置": {
                            "endpoint": "POST /robotic_control/collaborative_arm/move_to",
                            "description": "协作臂移动到指定位置",
                            "parameters": {
                                "x": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "x方向位置"
                                },
                                "y": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "y方向位置"
                                },
                                "z": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "z方向位置"
                                },
                                "roll": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角roll"
                                },
                                "yaw": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角yaw"
                                },
                                "pitch": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "方向角pitch"
                                }
                            },
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        },
                        "协作臂吸盘吸附": {
                            "endpoint": "POST /robotic_control/collaborative_arm/set_adsorption",
                            "description": "协作臂吸盘吸附控制",
                            "parameters": {
                                "value": {
                                    "type": "Float",
                                    "required": True,
                                    "description": "吸力大小，范围(0,1)"
                                }
                            },
                            "response": {
                                "code": "String",
                                "message": "String",
                                "data": {
                                    "status": "Bool"
                                }
                            }
                        }
                    }
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
            data = {}
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "success": False, 
                        "error": "Content-Type must be application/json"
                    }), 400
                
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                attribute = data.get('attribute', 0)
                
                self._log_api_call('/api/mapping/start', 'POST', data)
                
                result = self.navigator.start_mapping(attribute)
                message = "建图命令已发送" if result else "建图命令发送失败"
                
                self._log_api_call('/api/mapping/start', 'POST', data, result, message)
                
                return jsonify({
                    "success": result,
                    "message": message,
                    "seq": seq
                })
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/api/mapping/start', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({"success": False, "error": error_msg}), 500
        
        @self.app.route('/api/mapping/end', methods=['POST'])
        def end_mapping():
            """结束建图"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "success": False, 
                        "error": "Content-Type must be application/json"
                    }), 400
                
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                floor_index = data.get('floor_index', 0)
                pcdmap_index = data.get('pcdmap_index', 0)
                
                result = self.navigator.end_mapping(floor_index, pcdmap_index)
                return jsonify({
                    "success": result,
                    "message": "结束建图命令已发送" if result else "结束建图命令发送失败",
                    "seq": seq
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        
        # 导航控制
        @self.app.route('/api/navigation/start_loop', methods=['POST'])
        def start_loop_navigation():
            """开始导航"""
            try:
                data = request.get_json() or {}
                seq = data.get('seq', 'index:123;')
                self.navigator.start_relocation()
                self.navigator.start_navigation()
                self.navigator.pose_init()
                result = self.navigator.start_navigation_loop(seq)
                
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
                
                result = self.navigator.pause_navigation()
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
                
                result = self.navigator.recover_navigation()
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
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "success": False, 
                        "error": "Content-Type must be application/json"
                    }), 400
                
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
                
                self.navigator.add_node(node_name, x, y, z, yaw, False)
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
                
                result = self.navigator.add_node_at_current_pose(node_name, use_realtime, False)
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
                
                self.navigator.delete_node(node_ids)
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
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "success": False, 
                        "error": "Content-Type must be application/json"
                    }), 400
                
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
                
                self.navigator.add_edge(edge_name, start_node, end_node, dog_speed, False)
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
                
                self.navigator.delete_edge(edge_ids)
                return jsonify({
                    "success": True,
                    "message": f"边 {edge_ids} 删除命令已发送",
                    "deleted_edges": edge_ids
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
        

        
        
        # 相机数据获取
        @self.app.route('/api/camera/data', methods=['POST'])
        def get_camera_data():
            """获取相机数据"""
            data = {}
            try:
                data = request.get_json()
                if not data:
                    self._log_api_call('/api/camera/data', 'POST', {}, False, "缺少请求数据")
                    return jsonify({"success": False, "error": "缺少请求数据"}), 400
                
                camera_names = data.get('camera_names')
                camera_modes = data.get('camera_modes')
                
                if not camera_names:
                    self._log_api_call('/api/camera/data', 'POST', data, False, "缺少camera_names参数")
                    return jsonify({"success": False, "error": "缺少必要参数: camera_names"}), 400
                
                self._log_api_call('/api/camera/data', 'POST', data)
                
                result = self.navigator.get_camera_data(camera_names, camera_modes)
                success = result.get('code') == '000000'
                message = f"相机数据获取{'成功' if success else '失败'}: {result.get('message', '')}"
                
                self._log_api_call('/api/camera/data', 'POST', data, success, message)
                
                return jsonify({
                    "success": success,
                    "code": result.get('code'),
                    "message": result.get('message'),
                    "data": result.get('data', {}),
                    "camera_count": len(result.get('data', {})),
                    "timestamp": time.time()
                })
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/api/camera/data', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({"success": False, "error": error_msg}), 500
        
        # 机器人导航状态
        @self.app.route('/robotic_control/navigation/get_nav_state', methods=['GET'])
        def get_nav_state():
            """获取机器人导航状态"""
            try:
                self._log_api_call('/robotic_control/navigation/get_nav_state', 'GET')
                
                nav_state = self.navigator.get_nav_state()
                message = "导航状态获取成功"
                
                self._log_api_call('/robotic_control/navigation/get_nav_state', 'GET', {}, True, message)
                
                return jsonify({
                    "success": True,
                    "nav_state": nav_state,
                    "message": message,
                    "timestamp": time.time()
                })
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/navigation/get_nav_state', 'GET', {}, False, f"异常: {error_msg}")
                return jsonify({"success": False, "error": error_msg}), 500
        
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
                result = self.navigator.pose_init(trans_tuple, quat_tuple)
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
                
                result = self.navigator.start_relocation()
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
        
        # 自主建图
        @self.app.route('/robotic_control/navigation/autonomous_mapping', methods=['POST'])
        def autonomous_mapping():
            """自主建图"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 获取请求参数
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/navigation/autonomous_mapping', 'POST', data)
                
                # 必需参数验证
                if 'command' not in data:
                    return jsonify({
                        "code": "000001",
                        "message": "缺少必需参数: command",
                        "data": {
                            "status": False
                        }
                    }), 400
                if 'save' not in data:
                    return jsonify({
                        "code": "000001",
                        "message": "缺少必需参数: save",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 提取参数
                command = data.get('command')  # 1表示开始，0表示终止
                save = data.get('save')  # 是否保存所建地图
                save_path = data.get('save_path', None)  # 保存地图的路径（可选）
                max_time_out = data.get('max_time_out', 3600)  # 设置最长自主建图时间（秒）
                
                # 参数验证
                if command not in [0, 1]:
                    return jsonify({
                        "code": "000001",
                        "message": "command参数必须为0或1",
                        "data": {
                            "status": False
                        }
                    }), 400
                if not isinstance(save, bool):
                    return jsonify({
                        "code": "000001",
                        "message": "save参数必须为布尔值",
                        "data": {
                            "status": False
                        }
                    }), 400
                if max_time_out <= 0:
                    return jsonify({
                        "code": "000001",
                        "message": "max_time_out参数必须大于0",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 记录参数
                self.logger.info(f"自主建图参数: command={command}, save={save}, "
                               f"save_path={save_path}, max_time_out={max_time_out}s")
                
                # 根据command参数执行相应操作
                if command == 1:
                    # 开始自主建图
                    try:
                        self.navigator.start_mapping()
                        message = "自主建图已开始"
                        
                        # 如果设置了超时时间，启动定时器自动关闭建图
                        if max_time_out > 0:
                            self.logger.info(f"设置建图超时时间: {max_time_out}秒")
                            
                            # 取消之前的超时线程（如果存在）
                            if 'mapping_timeout' in self.timeout_threads:
                                self.logger.info("取消之前的建图超时线程")
                            
                            # 启动后台线程来处理超时
                            import threading
                            def auto_stop_mapping():
                                try:
                                    time.sleep(max_time_out)
                                    self.logger.info(f"建图超时时间到达({max_time_out}秒)，自动停止建图")
                                    
                                    # 停止建图
                                    self.navigator.end_mapping()
                                    
                                    # 如果需要保存地图
                                    if save:
                                        try:
                                            if save_path:
                                                self.navigator.save_accumulated_cloud(save_path)
                                            else:
                                                self.navigator.save_accumulated_cloud("autonomous_mapping_result.pcd")
                                            self.logger.info("超时停止建图后，地图已保存")
                                        except Exception as save_error:
                                            self.logger.error(f"超时停止建图后保存地图失败: {save_error}")
                                    
                                    # 清理超时线程记录
                                    if 'mapping_timeout' in self.timeout_threads:
                                        del self.timeout_threads['mapping_timeout']
                                        
                                except Exception as e:
                                    self.logger.error(f"自动停止建图失败: {e}")
                            
                            # 启动超时线程
                            timeout_thread = threading.Thread(target=auto_stop_mapping, daemon=True)
                            timeout_thread.start()
                            
                            # 记录超时线程
                            self.timeout_threads['mapping_timeout'] = {
                                'thread': timeout_thread,
                                'start_time': time.time(),
                                'timeout': max_time_out,
                                'save': save,
                                'save_path': save_path
                            }
                            
                            message += f"，将在{max_time_out}秒后自动停止"
                        
                        # 记录成功日志
                        self._log_api_call('/robotic_control/navigation/autonomous_mapping', 'POST', data, True, message)
                        
                        return jsonify({
                            "code": "000000",
                            "message": message,
                            "data": {
                                "status": True
                            }
                        })
                        
                    except Exception as e:
                        error_msg = f"开始自主建图失败: {str(e)}"
                        self.logger.error(error_msg)
                        self._log_api_call('/robotic_control/navigation/autonomous_mapping', 'POST', data, False, error_msg)
                        
                        return jsonify({
                            "code": "000002",
                            "message": error_msg,
                            "data": {
                                "status": False
                            }
                        }), 500
                        
                elif command == 0:
                    # 终止自主建图
                    try:
                        self.navigator.end_mapping()
                        message = "自主建图已终止"
                        
                        # 取消超时线程（如果存在）
                        if 'mapping_timeout' in self.timeout_threads:
                            self.logger.info("手动停止建图，取消超时线程")
                            del self.timeout_threads['mapping_timeout']
                        
                        # 如果需要保存地图
                        if save:
                            try:
                                if save_path:
                                    # 使用指定路径保存
                                    self.navigator.save_accumulated_cloud(save_path)
                                else:
                                    # 使用默认路径保存
                                    self.navigator.save_accumulated_cloud("autonomous_mapping_result.pcd")
                                message += "，地图已保存"
                            except Exception as save_error:
                                self.logger.error(f"保存地图失败: {save_error}")
                                message += "，但保存地图失败"
                        
                        # 记录成功日志
                        self._log_api_call('/robotic_control/navigation/autonomous_mapping', 'POST', data, True, message)
                        
                        return jsonify({
                            "code": "000000",
                            "message": message,
                            "data": {
                                "status": True
                            }
                        })
                        
                    except Exception as e:
                        error_msg = f"终止自主建图失败: {str(e)}"
                        self.logger.error(error_msg)
                        self._log_api_call('/robotic_control/navigation/autonomous_mapping', 'POST', data, False, error_msg)
                        
                        return jsonify({
                            "code": "000002",
                            "message": error_msg,
                            "data": {
                                "status": False
                            }
                        }), 500
                
            except Exception as e:
                error_msg = f"自主建图操作失败: {str(e)}"
                self.logger.error(error_msg)
                self._log_api_call('/robotic_control/navigation/autonomous_mapping', 'POST', data, False, error_msg)
                
                return jsonify({
                    "code": "000002",
                    "message": error_msg,
                    "data": {
                        "status": False
                    }
                }), 500
        
        # 定点导航
        @self.app.route('/robotic_control/navigation/fixed_point_nav', methods=['POST'])
        def fixed_point_navigation():
            """定点导航"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 获取请求参数
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/navigation/fixed_point_nav', 'POST', data)
                
                # 提取参数
                goal_coordinates_data = data.get('goal_coordinates', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})  # 目标点坐标
                map_name = data.get('map', 'default')  # String 使用的地图名称
                goal_node_id = data.get('goal_node_id', None)  # Int 目标点ID
                
                # 参数验证
                if not isinstance(goal_coordinates_data, dict):
                    return jsonify({
                        "code": "000001",
                        "message": "goal_coordinates必须是对象格式",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                try:
                    x = float(goal_coordinates_data.get('x', 0.0))
                    y = float(goal_coordinates_data.get('y', 0.0))
                    goal_yaw = float(goal_coordinates_data.get('yaw', 0.0))
                except (ValueError, TypeError):
                    return jsonify({
                        "code": "000001",
                        "message": "goal_coordinates中的坐标值必须是数字",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if not isinstance(goal_yaw, (int, float)):
                    return jsonify({
                        "code": "000001",
                        "message": "goal_yaw必须是数字",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 记录参数
                self.logger.info(f"定点导航参数: goal_coordinates=[{x}, {y}], map={map_name}, goal_yaw={goal_yaw}")
                
                # 获取当前位姿
                try:
                    current_pose = self.navigator.get_current_pose()
                    if current_pose is None:
                        return jsonify({
                            "code": "000001",
                            "message": "无法获取当前位姿，请先初始化位姿",
                            "data": {
                                "status": False
                            }
                        }), 400
                except Exception as pose_error:
                    return jsonify({
                        "code": "000002",
                        "message": f"获取当前位姿失败: {str(pose_error)}",
                        "data": {
                            "status": False
                        }
                    }), 500
                
                # 执行定点导航
                try:
                    # 这里需要根据实际的导航系统实现来调用相应的方法
                    # 假设navigator有navigate_to_point方法
                    success = self.navigator.navigate_to_point(
                        x=x,
                        y=y,
                        yaw=goal_yaw,
                        goal_node_id=goal_node_id,
                        map_name=map_name
                    )
                    
                    if success:
                        message = f"定点导航已启动，目标点: ({x:.2f}, {y:.2f}), 目标点ID: {goal_node_id}, 目标角度: {goal_yaw:.2f}°"
                        self.logger.info(message)
                        
                        # 记录成功日志
                        self._log_api_call('/robotic_control/navigation/fixed_point_nav', 'POST', data, True, message)
                        
                        return jsonify({
                            "code": "000000",
                            "message": message,
                            "data": {
                                "status": True
                            }
                        })
                    else:
                        error_msg = "定点导航启动失败"
                        self.logger.error(error_msg)
                        self._log_api_call('/robotic_control/navigation/fixed_point_nav', 'POST', data, False, error_msg)
                        
                        return jsonify({
                            "code": "000002",
                            "message": error_msg,
                            "data": {
                                "status": False
                            }
                        }), 500
                        
                except Exception as nav_error:
                    error_msg = f"导航执行失败: {str(nav_error)}"
                    self.logger.error(error_msg)
                    self._log_api_call('/robotic_control/navigation/fixed_point_nav', 'POST', data, False, error_msg)
                    
                    return jsonify({
                        "code": "000002",
                        "message": error_msg,
                        "data": {
                            "status": False
                        }
                    }), 500
                
            except Exception as e:
                error_msg = f"定点导航操作失败: {str(e)}"
                self.logger.error(error_msg)
                self._log_api_call('/robotic_control/navigation/fixed_point_nav', 'POST', data, False, error_msg)
                
                return jsonify({
                    "code": "000002",
                    "message": error_msg,
                    "data": {
                        "status": False
                    }
                }), 500
        
        @self.app.route('/robotic_control/navigation/go_home', methods=['POST'])
        def go_home():
            """一键返回"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 获取请求参数
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/navigation/go_home', 'POST', data)
                
                # 提取参数
                home_coordinates = data.get('home', None)  # List<Float> 导航图中的返航点
                
                # 参数验证（home参数是可选的）
                if home_coordinates is not None:
                    if not isinstance(home_coordinates, list):
                        return jsonify({
                            "code": "000001",
                            "message": "home参数必须是数组格式",
                            "data": {
                                "status": False
                            }
                        }), 400
                    
                    # 验证数组中的元素都是数字
                    try:
                        if len(home_coordinates) >= 2:
                            x = float(home_coordinates[0])
                            y = float(home_coordinates[1])
                            yaw = float(home_coordinates[2]) if len(home_coordinates) > 2 else 0.0
                        else:
                            return jsonify({
                                "code": "000001",
                                "message": "home参数至少需要包含x和y坐标",
                                "data": {
                                    "status": False
                                }
                            }), 400
                    except (ValueError, TypeError):
                        return jsonify({
                            "code": "000001",
                            "message": "home参数中的坐标值必须是数字",
                            "data": {
                                "status": False
                            }
                        }), 400
                
                # 记录参数
                if home_coordinates:
                    self.logger.info(f"返航参数: home={home_coordinates}")
                else:
                    self.logger.info("使用默认返航点")
                
                # 执行返航操作
                try:
                    if home_coordinates:
                        # 如果设置了新的返航点，导航到指定位置
                        success = self.navigator.navigate_to_point(
                            x=x,
                            y=y,
                            yaw=yaw
                        )
                        if success:
                            message = f"返航已启动，目标点: ({x:.2f}, {y:.2f}), 角度: {yaw:.2f}°"
                        else:
                            error_msg = "返航启动失败"
                            self.logger.error(error_msg)
                            self._log_api_call('/robotic_control/navigation/go_home', 'POST', data, False, error_msg)
                            
                            return jsonify({
                                "code": "000002",
                                "message": error_msg,
                                "data": {
                                    "status": False
                                }
                            }), 500
                    else:
                        # 使用默认返航点
                        self.navigator.go_home()
                        message = "返航已启动，使用默认返航点"
                    
                    # 记录成功日志
                    self.logger.info(message)
                    self._log_api_call('/robotic_control/navigation/go_home', 'POST', data, True, message)
                    
                    return jsonify({
                        "code": "000000",
                        "message": message,
                        "data": {
                            "status": True
                        }
                    })
                    
                except Exception as nav_error:
                    error_msg = f"返航执行失败: {str(nav_error)}"
                    self.logger.error(error_msg)
                    self._log_api_call('/robotic_control/navigation/go_home', 'POST', data, False, error_msg)
                    
                    return jsonify({
                        "code": "000002",
                        "message": error_msg,
                        "data": {
                            "status": False
                        }
                    }), 500
                
            except Exception as e:
                error_msg = f"返航操作失败: {str(e)}"
                self.logger.error(error_msg)
                self._log_api_call('/robotic_control/navigation/go_home', 'POST', data, False, error_msg)
                
                return jsonify({
                    "code": "000002",
                    "message": error_msg,
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/navigation/set_auto_nav', methods=['POST'])
        def set_auto_nav():
            """设置自动导航"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 获取请求参数
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/navigation/set_auto_nav', 'POST', data)
                
                # 提取参数
                map_name = data.get('map', 'default')  # String 使用的导航地图
                area = data.get('area', [])  # List<Float> 划定的导航区域范围
                path_point = data.get('path_point', [])  # List<Float> 设置的巡逻路径点
                
                # 参数验证
                if map_name is not None and not isinstance(map_name, str):
                    return jsonify({
                        "code": "000001",
                        "message": "map参数必须是字符串类型",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if area is not None and not isinstance(area, list):
                    return jsonify({
                        "code": "000001",
                        "message": "area参数必须是数组格式",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if path_point is not None and not isinstance(path_point, list):
                    return jsonify({
                        "code": "000001",
                        "message": "path_point参数必须是数组格式",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 验证数组中的元素都是数字（如果提供了数组）
                if area:
                    try:
                        for i, value in enumerate(area):
                            float(value)
                    except (ValueError, TypeError):
                        return jsonify({
                            "code": "000001",
                            "message": f"area参数第{i+1}个元素必须是数字类型",
                            "data": {
                                "status": False
                            }
                        }), 400
                
                if path_point:
                    try:
                        for i, value in enumerate(path_point):
                            float(value)
                    except (ValueError, TypeError):
                        return jsonify({
                            "code": "000001",
                            "message": f"path_point参数第{i+1}个元素必须是数字类型",
                            "data": {
                                "status": False
                            }
                        }), 400
                
                # 记录参数
                self.logger.info(f"设置自动导航参数: map={map_name}, area={area}, path_point={path_point}")
                
                # 调用navigator方法设置自动导航
                try:
                    result = self.navigator.set_auto_nav(map_name=map_name, area=area, path_point=path_point)
                    
                    if result:
                        message = "自动导航参数设置成功"
                        self.logger.info(message)
                        
                        # 记录成功日志
                        self._log_api_call('/robotic_control/navigation/set_auto_nav', 'POST', data, True, message)
                        
                        return jsonify({
                            "code": "000000",
                            "message": message,
                            "data": {
                                "status": True
                            }
                        })
                    else:
                        error_msg = "自动导航参数设置失败"
                        self.logger.error(error_msg)
                        self._log_api_call('/robotic_control/navigation/set_auto_nav', 'POST', data, False, error_msg)
                        
                        return jsonify({
                            "code": "000002",
                            "message": error_msg,
                            "data": {
                                "status": False
                            }
                        }), 500
                        
                except Exception as nav_error:
                    error_msg = f"自动导航设置执行失败: {str(nav_error)}"
                    self.logger.error(error_msg)
                    self._log_api_call('/robotic_control/navigation/set_auto_nav', 'POST', data, False, error_msg)
                    
                    return jsonify({
                        "code": "000002",
                        "message": error_msg,
                        "data": {
                            "status": False
                        }
                    }), 500
                
            except Exception as e:
                error_msg = f"设置自动导航操作失败: {str(e)}"
                self.logger.error(error_msg)
                self._log_api_call('/robotic_control/navigation/set_auto_nav', 'POST', data, False, error_msg)
                
                return jsonify({
                    "code": "000002",
                    "message": error_msg,
                    "data": {
                        "status": False
                    }
                }), 500

        # ==================== 节点和边管理扩展功能 ====================
        
        @self.app.route('/api/nodes/delete_all', methods=['DELETE'])
        def delete_all_nodes():
            """删除所有节点"""
            try:
                self._log_api_call('/api/nodes/delete_all', 'DELETE')
                result = self.navigator.delete_all_nodes()
                return jsonify({
                    "success": True,
                    "message": "所有节点删除成功",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/nodes/delete_all', 'DELETE', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/edges/delete_all', methods=['DELETE'])
        def delete_all_edges():
            """删除所有边"""
            try:
                self._log_api_call('/api/edges/delete_all', 'DELETE')
                result = self.navigator.delete_all_edges()
                return jsonify({
                    "success": True,
                    "message": "所有边删除成功",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/edges/delete_all', 'DELETE', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/nodes/query', methods=['POST'])
        def query_nodes():
            """查询节点"""
            try:
                data = request.get_json() or {}
                attribute = data.get('attribute', 1)
                
                self._log_api_call('/api/nodes/query', 'POST', data)
                success, result = self.navigator.query_node(attribute)
                
                return jsonify({
                    "success": success,
                    "message": "节点查询完成",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/nodes/query', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/edges/query', methods=['POST'])
        def query_edges():
            """查询边"""
            try:
                data = request.get_json() or {}
                attribute = data.get('attribute', 2)
                
                self._log_api_call('/api/edges/query', 'POST', data)
                success, result = self.navigator.query_edge(attribute)
                
                return jsonify({
                    "success": success,
                    "message": "边查询完成",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/edges/query', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/network/overview', methods=['GET'])
        def network_overview():
            """网络概览分析"""
            try:
                self._log_api_call('/api/network/overview', 'GET')
                
                # 获取节点和边数据
                nodes_success, nodes_result = self.navigator.query_node(1)
                edges_success, edges_result = self.navigator.query_edge(2)
                
                if not nodes_success or not edges_success:
                    return jsonify({
                        "success": False,
                        "error": "无法获取网络数据"
                    }), 500
                
                # 这里可以添加网络分析逻辑
                overview = {
                    "total_nodes": nodes_result,
                    "total_edges": edges_result,
                    "timestamp": time.time()
                }
                
                return jsonify({
                    "success": True,
                    "message": "网络概览分析完成",
                    "overview": overview
                })
            except Exception as e:
                self._log_api_call('/api/network/overview', 'GET', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/path/analysis', methods=['POST'])
        def path_analysis():
            """路径分析"""
            try:
                data = request.get_json() or {}
                start_node = data.get('start_node')
                end_node = data.get('end_node')
                
                self._log_api_call('/api/path/analysis', 'POST', data)
                
                if start_node is None or end_node is None:
                    return jsonify({
                        "success": False,
                        "error": "需要提供 start_node 和 end_node 参数"
                    }), 400
                
                # 这里可以添加路径分析逻辑
                analysis = {
                    "start_node": start_node,
                    "end_node": end_node,
                    "paths_found": 0,
                    "shortest_path": [],
                    "timestamp": time.time()
                }
                
                return jsonify({
                    "success": True,
                    "message": "路径分析完成",
                    "analysis": analysis
                })
            except Exception as e:
                self._log_api_call('/api/path/analysis', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        # ==================== 内部存储管理 ====================
        
        @self.app.route('/api/internal/nodes/publish', methods=['POST'])
        def publish_all_nodes():
            """发布所有内部存储的节点"""
            try:
                self._log_api_call('/api/internal/nodes/publish', 'POST')
                result = self.navigator.publish_all_nodes()
                return jsonify({
                    "success": True,
                    "message": "所有内部节点发布成功",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/internal/nodes/publish', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/internal/edges/publish', methods=['POST'])
        def publish_all_edges():
            """发布所有内部存储的边"""
            try:
                self._log_api_call('/api/internal/edges/publish', 'POST')
                result = self.navigator.publish_all_edges()
                return jsonify({
                    "success": True,
                    "message": "所有内部边发布成功",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/internal/edges/publish', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/internal/publish_all', methods=['POST'])
        def publish_all_nodes_and_edges():
            """发布所有内部存储的节点和边"""
            try:
                self._log_api_call('/api/internal/publish_all', 'POST')
                node_result, edge_result = self.navigator.publish_all_nodes_and_edges()
                return jsonify({
                    "success": True,
                    "message": "所有内部节点和边发布成功",
                    "node_result": node_result,
                    "edge_result": edge_result
                })
            except Exception as e:
                self._log_api_call('/api/internal/publish_all', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/internal/clear', methods=['POST'])
        def clear_internal_storage():
            """清除内部存储"""
            try:
                self._log_api_call('/api/internal/clear', 'POST')
                self.navigator.clear_internal_storage()
                return jsonify({
                    "success": True,
                    "message": "内部存储清除成功"
                })
            except Exception as e:
                self._log_api_call('/api/internal/clear', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/internal/info', methods=['GET'])
        def get_internal_storage_info():
            """获取内部存储信息"""
            try:
                self._log_api_call('/api/internal/info', 'GET')
                info = self.navigator.get_internal_storage_info()
                return jsonify({
                    "success": True,
                    "message": "内部存储信息获取成功",
                    "info": info
                })
            except Exception as e:
                self._log_api_call('/api/internal/info', 'GET', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        # ==================== 自动收集功能 ====================
        
        @self.app.route('/api/auto_collect/node', methods=['POST'])
        def auto_collect_node():
            """自动收集当前位姿的节点"""
            try:
                data = request.get_json() or {}
                auto_connect = data.get('auto_connect', True)
                
                self._log_api_call('/api/auto_collect/node', 'POST', data)
                success, result = self.navigator.add_node_at_current_pose_auto_collect(auto_connect=auto_connect)
                
                return jsonify({
                    "success": success,
                    "message": "自动收集节点完成",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/auto_collect/node', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/auto_collect/save', methods=['POST'])
        def collect_and_save_nodes_edges():
            """收集并保存所有节点和边"""
            try:
                data = request.get_json() or {}
                clear_after_save = data.get('clear_after_save', True)
                
                self._log_api_call('/api/auto_collect/save', 'POST', data)
                node_result, edge_result = self.navigator.collect_and_save_nodes_edges(clear_after_save)
                
                return jsonify({
                    "success": True,
                    "message": "收集并保存节点和边完成",
                    "node_result": node_result,
                    "edge_result": edge_result
                })
            except Exception as e:
                self._log_api_call('/api/auto_collect/save', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/auto_collect/loop', methods=['POST'])
        def auto_collect_loop():
            """自动收集循环"""
            try:
                data = request.get_json() or {}
                node_interval = data.get('node_interval', 2.0)
                max_nodes = data.get('max_nodes', 10)
                
                self._log_api_call('/api/auto_collect/loop', 'POST', data)
                self.navigator.auto_collect_loop(node_interval, max_nodes)
                
                return jsonify({
                    "success": True,
                    "message": "自动收集循环启动",
                    "node_interval": node_interval,
                    "max_nodes": max_nodes
                })
            except Exception as e:
                self._log_api_call('/api/auto_collect/loop', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/auto_collect/prepare', methods=['POST'])
        def prepare_for_collection():
            """准备收集"""
            try:
                self._log_api_call('/api/auto_collect/prepare', 'POST')
                self.navigator.prepare_for_collection()
                return jsonify({
                    "success": True,
                    "message": "准备收集完成"
                })
            except Exception as e:
                self._log_api_call('/api/auto_collect/prepare', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/auto_collect/clear_and_map', methods=['POST'])
        def clear_and_start_mapping():
            """清除并开始映射"""
            try:
                self._log_api_call('/api/auto_collect/clear_and_map', 'POST')
                self.navigator.clear_collection_and_start_mapping()
                return jsonify({
                    "success": True,
                    "message": "清除并开始映射完成"
                })
            except Exception as e:
                self._log_api_call('/api/auto_collect/clear_and_map', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/auto_collect/status', methods=['GET'])
        def get_collection_status():
            """获取收集状态"""
            try:
                self._log_api_call('/api/auto_collect/status', 'GET')
                status = self.navigator.get_collection_status()
                return jsonify({
                    "success": True,
                    "message": "收集状态获取成功",
                    "status": status
                })
            except Exception as e:
                self._log_api_call('/api/auto_collect/status', 'GET', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        # ==================== Notice 缓存控制 ====================
        
        @self.app.route('/api/notice_cache/configure', methods=['POST'])
        def configure_notice_cache():
            """配置 notice 缓存"""
            try:
                data = request.get_json() or {}
                cache_duration = data.get('cache_duration', 2.0)
                auto_cache = data.get('auto_cache', True)
                
                self._log_api_call('/api/notice_cache/configure', 'POST', data)
                config = self.navigator.configure_notice_cache(cache_duration, auto_cache)
                
                return jsonify({
                    "success": True,
                    "message": "Notice 缓存配置成功",
                    "config": config
                })
            except Exception as e:
                self._log_api_call('/api/notice_cache/configure', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/notice_cache/status', methods=['GET'])
        def get_notice_cache_status():
            """获取 notice 缓存状态"""
            try:
                self._log_api_call('/api/notice_cache/status', 'GET')
                status = self.navigator.get_notice_cache_status()
                return jsonify({
                    "success": True,
                    "message": "Notice 缓存状态获取成功",
                    "status": status
                })
            except Exception as e:
                self._log_api_call('/api/notice_cache/status', 'GET', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/notice_cache/test', methods=['POST'])
        def test_notice_cache():
            """测试 notice 缓存"""
            try:
                data = request.get_json() or {}
                duration = data.get('duration', 3.0)
                
                self._log_api_call('/api/notice_cache/test', 'POST', data)
                results = self.navigator.test_notice_cache(duration)
                
                return jsonify({
                    "success": True,
                    "message": "Notice 缓存测试完成",
                    "results": results
                })
            except Exception as e:
                self._log_api_call('/api/notice_cache/test', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        # ==================== 其他功能 ====================
        
        @self.app.route('/api/nodes/close_all', methods=['POST'])
        def close_all_nodes():
            """关闭所有节点"""
            try:
                self._log_api_call('/api/nodes/close_all', 'POST')
                result = self.navigator.close_all_nodes()
                return jsonify({
                    "success": True,
                    "message": "所有节点关闭成功",
                    "result": result
                })
            except Exception as e:
                self._log_api_call('/api/nodes/close_all', 'POST', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/cloud/info', methods=['GET'])
        def get_cloud_info():
            """获取云信息"""
            try:
                self._log_api_call('/api/cloud/info', 'GET')
                cloud_size = self.navigator.get_cloud_size()
                traj_size = self.navigator.get_trajectory_size()
                total_size = self.navigator.get_total_cloud_size()
                
                info = {
                    "environment_cloud_points": cloud_size,
                    "trajectory_cloud_points": traj_size,
                    "total_cloud_points": total_size
                }
                
                return jsonify({
                    "success": True,
                    "message": "云信息获取成功",
                    "info": info
                })
            except Exception as e:
                self._log_api_call('/api/cloud/info', 'GET', success=False, message=str(e))
                return jsonify({"success": False, "error": str(e)}), 500
        
        @self.app.route('/api/downsample/configure', methods=['POST'])
        def set_downsample_parameters():
            """设置下采样参数"""
            try:
                data = request.get_json() or {}
                max_size = data.get('max_size', 100000)
                voxel_size = data.get('voxel_size', 0.05)
                
                self._log_api_call('/api/downsample/configure', 'POST', data)
                self.navigator.set_downsample_parameters(max_size, voxel_size)
                
                return jsonify({
                    "success": True,
                    "message": "下采样参数设置成功",
                    "max_size": max_size,
                    "voxel_size": voxel_size
                })
            except Exception as e:
                self._log_api_call('/api/downsample/configure', 'POST', success=False, message=str(e))
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
                
                # 添加超时状态信息
                timeout_info = {}
                if 'mapping_timeout' in self.timeout_threads:
                    timeout_data = self.timeout_threads['mapping_timeout']
                    elapsed_time = time.time() - timeout_data['start_time']
                    remaining_time = max(0, timeout_data['timeout'] - elapsed_time)
                    timeout_info = {
                        "mapping_timeout_active": True,
                        "timeout_duration": timeout_data['timeout'],
                        "elapsed_time": elapsed_time,
                        "remaining_time": remaining_time,
                        "will_save": timeout_data['save'],
                        "save_path": timeout_data['save_path']
                    }
                else:
                    timeout_info = {
                        "mapping_timeout_active": False
                    }
                
                status.update(timeout_info)
                
                return jsonify({
                    "success": True,
                    "status": status,
                    "message": "系统状态获取成功"
                })
            except Exception as e:
                return jsonify({"success": False, "error": str(e)}), 500
    
        # 机械臂api
        @self.app.route('/robotic_control/robotic_arm/get_pose', methods=['POST'])
        def get_robotic_arm_pose():
            """获取机械臂末端位姿"""
            try:
                self._log_api_call('/robotic_control/robotic_arm/get_pose', 'POST')
                
                # 获取机械臂末端位姿
                arm_pose = {}
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "机械臂位姿获取成功",
                    "data": {
                        "status": True,
                        "x": arm_pose.get('x', 0.0),
                        "y": arm_pose.get('y', 0.0),
                        "z": arm_pose.get('z', 0.0),
                        "roll": arm_pose.get('roll', 0.0),
                        "yaw": arm_pose.get('yaw', 0.0),
                        "pitch": arm_pose.get('pitch', 0.0)
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/robotic_arm/get_pose', 'POST', {}, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"获取机械臂位姿失败: {error_msg}",
                    "data": {
                        "status": False,
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "roll": 0.0,
                        "yaw": 0.0,
                        "pitch": 0.0
                    }
                }), 500

        @self.app.route('/robotic_control/robotic_arm/go_home', methods=['POST'])
        def robotic_arm_go_home():
            """机械臂返回默认位置"""
            try:
                self._log_api_call('/robotic_control/robotic_arm/go_home', 'POST')
                
                # TODO: 实现机械臂返回默认位置的具体逻辑
                # 这里应该调用实际的机械臂控制接口
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "机械臂返回默认位置成功",
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/robotic_arm/go_home', 'POST', {}, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"机械臂返回默认位置失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/open_door', methods=['POST'])
        def robotic_arm_open_door():
            """机械臂开关门"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/open_door', 'POST', data)
                
                # 获取必需参数
                open_door = data.get('open')
                direction = data.get('direction')
                
                # 参数验证
                if open_door is None:
                    return jsonify({
                        "code": "000001",
                        "message": "缺少必需参数: open",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if direction is None:
                    return jsonify({
                        "code": "000001",
                        "message": "缺少必需参数: direction",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 验证参数类型和值
                if not isinstance(open_door, bool):
                    return jsonify({
                        "code": "000001",
                        "message": "open参数必须是布尔值",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if not isinstance(direction, str) or direction not in ['left', 'right']:
                    return jsonify({
                        "code": "000001",
                        "message": "direction参数必须是字符串，且值为'left'或'right'",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # TODO: 实现机械臂开关门的具体逻辑
                # 这里应该调用实际的机械臂控制接口
                # 根据open_door和direction参数执行相应的开关门动作
                
                action = "开门" if open_door else "关门"
                message = f"机械臂{direction}方向{action}任务执行成功"
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": message,
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/open_door', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"机械臂开关门失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/robotic_arm/move_to', methods=['POST'])
        def robotic_arm_move_to():
            """机械臂移动到指定位置"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/robotic_arm/move_to', 'POST', data)
                
                # 获取必需参数
                x = data.get('x')
                y = data.get('y')
                z = data.get('z')
                roll = data.get('roll')
                yaw = data.get('yaw')
                pitch = data.get('pitch')
                
                # 参数验证
                required_params = {
                    'x': x,
                    'y': y,
                    'z': z,
                    'roll': roll,
                    'yaw': yaw,
                    'pitch': pitch
                }
                
                missing_params = []
                invalid_params = []
                
                for param_name, param_value in required_params.items():
                    if param_value is None:
                        missing_params.append(param_name)
                    elif not isinstance(param_value, (int, float)):
                        invalid_params.append(f"{param_name}必须是数字类型")
                
                # 返回参数错误信息
                if missing_params:
                    return jsonify({
                        "code": "000001",
                        "message": f"缺少必需参数: {', '.join(missing_params)}",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if invalid_params:
                    return jsonify({
                        "code": "000001",
                        "message": f"参数类型错误: {'; '.join(invalid_params)}",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 转换为浮点数
                try:
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    roll = float(roll)
                    yaw = float(yaw)
                    pitch = float(pitch)
                except (ValueError, TypeError):
                    return jsonify({
                        "code": "000001",
                        "message": "坐标参数无法转换为数字",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # TODO: 实现机械臂移动到指定位置的具体逻辑
                # 这里应该调用实际的机械臂控制接口
                # 根据x, y, z, roll, yaw, pitch参数执行机械臂移动
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "机械臂移动到指定位置成功",
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/robotic_arm/move_to', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"机械臂移动到指定位置失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/robotic_arm/ee_move_to', methods=['POST'])
        def robotic_arm_ee_move_to():
            """机械臂末端移动到指定位置"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/robotic_arm/ee_move_to', 'POST', data)
                
                # 获取必需参数
                x = data.get('x')
                y = data.get('y')
                z = data.get('z')
                roll = data.get('roll')
                yaw = data.get('yaw')
                pitch = data.get('pitch')
                
                # 参数验证
                required_params = {
                    'x': x,
                    'y': y,
                    'z': z,
                    'roll': roll,
                    'yaw': yaw,
                    'pitch': pitch
                }
                
                missing_params = []
                invalid_params = []
                
                for param_name, param_value in required_params.items():
                    if param_value is None:
                        missing_params.append(param_name)
                    elif not isinstance(param_value, (int, float)):
                        invalid_params.append(f"{param_name}必须是数字类型")
                
                # 返回参数错误信息
                if missing_params:
                    return jsonify({
                        "code": "000001",
                        "message": f"缺少必需参数: {', '.join(missing_params)}",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if invalid_params:
                    return jsonify({
                        "code": "000001",
                        "message": f"参数类型错误: {'; '.join(invalid_params)}",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 转换为浮点数
                try:
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    roll = float(roll)
                    yaw = float(yaw)
                    pitch = float(pitch)
                except (ValueError, TypeError):
                    return jsonify({
                        "code": "000001",
                        "message": "坐标参数无法转换为数字",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # TODO: 实现机械臂末端移动到指定位置的具体逻辑
                # 这里应该调用实际的机械臂控制接口
                # 根据x, y, z, roll, yaw, pitch参数执行机械臂末端移动
                # 注意：仅移动末端关节，保持除末端以外的关节不变，用于微调位姿
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "机械臂末端移动到指定位置成功",
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/robotic_arm/ee_move_to', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"机械臂末端移动到指定位置失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/robotic_arm/get_camera', methods=['POST'])
        def robotic_arm_get_camera():
            """机械臂末端相机获取"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False,
                            "images": []
                        }
                    }), 400
                
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/robotic_arm/get_camera', 'POST', data)
                
                # 获取可选参数
                rgb = data.get('rgb', False)
                depth = data.get('depth', False)
                
                # 参数验证
                if not isinstance(rgb, bool):
                    return jsonify({
                        "code": "000001",
                        "message": "rgb参数必须是布尔值",
                        "data": {
                            "status": False,
                            "images": []
                        }
                    }), 400
                
                if not isinstance(depth, bool):
                    return jsonify({
                        "code": "000001",
                        "message": "depth参数必须是布尔值",
                        "data": {
                            "status": False,
                            "images": []
                        }
                    }), 400
                
                # 检查是否至少需要一种图像类型
                if not rgb and not depth:
                    return jsonify({
                        "code": "000001",
                        "message": "至少需要指定一种图像类型：rgb或depth",
                        "data": {
                            "status": False,
                            "images": []
                        }
                    }), 400
                
                # TODO: 实现机械臂末端相机获取的具体逻辑
                # 这里应该调用实际的机械臂相机接口
                # 根据rgb和depth参数获取相应的图像数据
                
                # 模拟返回的图像列表
                images = []
                if rgb:
                    images.append({
                        "type": "rgb",
                        "data": "base64_encoded_rgb_image_data",
                        "timestamp": time.time()
                    })
                
                if depth:
                    images.append({
                        "type": "depth",
                        "data": "base64_encoded_depth_image_data",
                        "timestamp": time.time()
                    })
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "机械臂末端相机获取成功",
                    "data": {
                        "status": True,
                        "images": images
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/robotic_arm/get_camera', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"机械臂末端相机获取失败: {error_msg}",
                    "data": {
                        "status": False,
                        "images": []
                    }
                }), 500

        @self.app.route('/robotic_control/robotic_arm/set_gripper', methods=['POST'])
        def robotic_arm_set_gripper():
            """机械臂夹爪开合"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/robotic_arm/set_gripper', 'POST', data)
                
                # 获取必需参数
                value = data.get('value')
                
                # 参数验证
                if value is None:
                    return jsonify({
                        "code": "000001",
                        "message": "缺少必需参数: value",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 验证参数类型
                if not isinstance(value, (int, float)):
                    return jsonify({
                        "code": "000001",
                        "message": "value参数必须是数字类型",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 转换为浮点数并验证范围
                try:
                    value = float(value)
                except (ValueError, TypeError):
                    return jsonify({
                        "code": "000001",
                        "message": "value参数无法转换为数字",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 验证范围 (0,1)
                if value <= 0 or value >= 1:
                    return jsonify({
                        "code": "000001",
                        "message": "value参数必须在范围(0,1)内",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # TODO: 实现机械臂夹爪开合的具体逻辑
                # 这里应该调用实际的机械臂夹爪控制接口
                # 根据value参数控制夹爪的开合角度
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "机械臂夹爪开合成功",
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/robotic_arm/set_gripper', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"机械臂夹爪开合失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/collaborative_arm/go_home', methods=['POST'])
        def collaborative_arm_go_home():
            """协作臂返回默认位置"""
            try:
                self._log_api_call('/robotic_control/collaborative_arm/go_home', 'POST')
                
                # TODO: 实现协作臂返回默认位置的具体逻辑
                # 这里应该调用实际的协作臂控制接口
                # 控制机器狗末端协作臂返回默认位置
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "协作臂返回默认位置成功",
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/collaborative_arm/go_home', 'POST', {}, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"协作臂返回默认位置失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/collaborative_arm/move_to', methods=['POST'])
        def collaborative_arm_move_to():
            """协作臂移动到指定位置"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/collaborative_arm/move_to', 'POST', data)
                
                # 获取必需参数
                x = data.get('x')
                y = data.get('y')
                z = data.get('z')
                roll = data.get('roll')
                yaw = data.get('yaw')
                pitch = data.get('pitch')
                
                # 参数验证
                required_params = {
                    'x': x,
                    'y': y,
                    'z': z,
                    'roll': roll,
                    'yaw': yaw,
                    'pitch': pitch
                }
                
                missing_params = []
                invalid_params = []
                
                for param_name, param_value in required_params.items():
                    if param_value is None:
                        missing_params.append(param_name)
                    elif not isinstance(param_value, (int, float)):
                        invalid_params.append(f"{param_name}必须是数字类型")
                
                # 返回参数错误信息
                if missing_params:
                    return jsonify({
                        "code": "000001",
                        "message": f"缺少必需参数: {', '.join(missing_params)}",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                if invalid_params:
                    return jsonify({
                        "code": "000001",
                        "message": f"参数类型错误: {'; '.join(invalid_params)}",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 转换为浮点数
                try:
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    roll = float(roll)
                    yaw = float(yaw)
                    pitch = float(pitch)
                except (ValueError, TypeError):
                    return jsonify({
                        "code": "000001",
                        "message": "坐标参数无法转换为数字",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # TODO: 实现协作臂移动到指定位置的具体逻辑
                # 这里应该调用实际的协作臂控制接口
                # 根据x, y, z, roll, yaw, pitch参数执行协作臂移动
                # 注意：协作臂通常具有安全特性和人机协作功能
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "协作臂移动到指定位置成功",
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/collaborative_arm/move_to', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"协作臂移动到指定位置失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500

        @self.app.route('/robotic_control/collaborative_arm/set_adsorption', methods=['POST'])
        def collaborative_arm_set_adsorption():
            """协作臂吸盘吸附"""
            try:
                # 检查 Content-Type
                if not request.is_json:
                    return jsonify({
                        "code": "000001",
                        "message": "Content-Type must be application/json",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                data = request.get_json() or {}
                self._log_api_call('/robotic_control/collaborative_arm/set_adsorption', 'POST', data)
                
                # 获取必需参数
                value = data.get('value')
                
                # 参数验证
                if value is None:
                    return jsonify({
                        "code": "000001",
                        "message": "缺少必需参数: value",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 验证参数类型
                if not isinstance(value, (int, float)):
                    return jsonify({
                        "code": "000001",
                        "message": "value参数必须是数字类型",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 转换为浮点数并验证范围
                try:
                    value = float(value)
                except (ValueError, TypeError):
                    return jsonify({
                        "code": "000001",
                        "message": "value参数无法转换为数字",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # 验证范围 (0,1)
                if value <= 0 or value >= 1:
                    return jsonify({
                        "code": "000001",
                        "message": "value参数必须在范围(0,1)内",
                        "data": {
                            "status": False
                        }
                    }), 400
                
                # TODO: 实现协作臂吸盘吸附的具体逻辑
                # 这里应该调用实际的协作臂吸盘控制接口
                # 根据value参数控制吸盘的吸力大小
                # 注意：value为0表示无吸力，value为1表示最大吸力
                
                # 返回成功响应
                return jsonify({
                    "code": "000000",
                    "message": "协作臂吸盘吸附设置成功",
                    "data": {
                        "status": True
                    }
                })
                
            except Exception as e:
                error_msg = str(e)
                self._log_api_call('/robotic_control/collaborative_arm/set_adsorption', 'POST', data, False, f"异常: {error_msg}")
                return jsonify({
                    "code": "000002",
                    "message": f"协作臂吸盘吸附设置失败: {error_msg}",
                    "data": {
                        "status": False
                    }
                }), 500


    def _start_ros_thread(self):
        """启动ROS2 spin线程"""
        self.logger.info("启动ROS2 spin线程")
        self.running = True
        while self.running:
            try:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)
            except Exception as e:
                self.logger.error(f"ROS spin错误: {e}")
                break
        self.logger.info("ROS2 spin线程已停止")
    
    def start(self):
        """启动HTTP服务器"""
        try:
            # 启动ROS2线程
            self.ros_thread = threading.Thread(target=self._start_ros_thread, daemon=True)
            self.ros_thread.start()
            self.logger.info("ROS2线程已启动")
            
            self.logger.info(f"🚀 HTTP Navigator Agent 启动中...")
            self.logger.info(f"📡 服务器地址: http://{self.host}:{self.port}")
            self.logger.info(f"📖 API文档: http://{self.host}:{self.port}/api/help")
            self.logger.info(f"🔍 健康检查: http://{self.host}:{self.port}/api/health")
            self.logger.info("✅ 服务器已启动，可以接收命令!")
            
            print(f"🚀 HTTP Navigator Agent 启动中...")
            print(f"📡 服务器地址: http://{self.host}:{self.port}")
            print(f"📖 API文档: http://{self.host}:{self.port}/api/help")
            print(f"🔍 健康检查: http://{self.host}:{self.port}/api/health")
            print("✅ 服务器已启动，可以接收命令!")
            
            # 启动Flask服务器
            self.app.run(host=self.host, port=self.port, debug=self.debug, threaded=True)
            
        except KeyboardInterrupt:
            self.logger.info("⚠️ 收到中断信号，正在关闭服务器...")
            print("\n⚠️ 收到中断信号，正在关闭服务器...")
            self.stop()
        except Exception as e:
            self.logger.error(f"❌ 服务器启动失败: {e}")
            print(f"❌ 服务器启动失败: {e}")
            self.stop()
    
    def stop(self):
        """停止服务器"""
        self.logger.info("正在停止HTTP Navigator Agent...")
        self.running = False
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=2.0)
            self.logger.info("ROS2线程已停止")
        
        if hasattr(self, 'navigator'):
            self.navigator.stop_visualization()
            self.navigator.destroy_node()
            self.logger.info("Navigator节点已销毁")
        
        rclpy.shutdown()
        self.logger.info("🛑 HTTP Navigator Agent 已停止")
        self.logger.info("=" * 50)
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
    print(f"调试模式: {'开启' if args.debug else '关闭'}")
    print("按 Ctrl+C 停止服务器")
    print("=" * 50)
    
    # 创建并启动agent
    try:
        agent = HTTPNavigatorAgent(host=args.host, port=args.port, debug=args.debug)
        agent.start()
    except Exception as e:
        print(f"❌ 启动失败: {e}")
        exit(1)


if __name__ == "__main__":
    main()
