#!/usr/bin/env python3
"""
Interactive HTTP Navigator Agent Test

基于 navigator_example.py 的键盘功能，提供交互式 HTTP API 测试界面
可以通过键盘命令测试 http_navigator_agent.py 的所有功能
"""

import requests
import json
import time
import sys
import os
from typing import Dict, Any, Optional
from dataclasses import dataclass

@dataclass
class Point2D:
    x: float
    y: float

class InteractiveHTTPTester:
    """交互式 HTTP API 测试器"""
    
    def __init__(self, base_url: str = "http://localhost:8080"):
        """
        初始化 HTTP 测试器
        
        Args:
            base_url: HTTP 服务器基础 URL
        """
        self.base_url = base_url
        self.session = requests.Session()
        self.session.headers.update({
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        })
        
        # 测试状态
        self.test_results = []
        self.current_pose = None
        self.nav_state = None
        
        print(f"🌐 HTTP Navigator Agent 交互式测试器")
        print(f"📡 服务器地址: {base_url}")
        print(f"📖 按 'H' 查看帮助，按 'Q' 退出")
        print("=" * 60)
    
    def make_request(self, method: str, endpoint: str, data: Optional[Dict] = {}) -> Dict:
        """
        发送 HTTP 请求
        
        Args:
            method: HTTP 方法 (GET, POST, DELETE)
            endpoint: API 端点
            data: 请求数据
            
        Returns:
            Dict: 响应数据
        """
        url = f"{self.base_url}{endpoint}"
        
        try:
            if method.upper() == 'GET':
                response = self.session.get(url)
            elif method.upper() == 'POST':
                headers = {'Content-Type': 'application/json'}
                response = self.session.post(url, json=data, headers=headers)
            elif method.upper() == 'DELETE':
                response = self.session.delete(url, json=data)
            else:
                return {'error': f'Unsupported method: {method}'}
            
            response.raise_for_status()
            return response.json()
            
        except requests.exceptions.ConnectionError:
            return {'error': f'连接失败: 无法连接到 {url}'}
        except requests.exceptions.Timeout:
            return {'error': f'请求超时: {url}'}
        except requests.exceptions.RequestException as e:
            return {'error': f'请求失败: {str(e)}'}
        except json.JSONDecodeError:
            return {'error': f'响应解析失败: {response.text}'}
    
    def get_user_input(self, prompt: str) -> str:
        """获取用户输入"""
        return input(prompt).strip()
    
    def get_int_input(self, prompt: str, default: int = 1) -> int:
        """获取整数输入"""
        try:
            user_input = self.get_user_input(f"{prompt} (默认: {default}): ")
            return int(user_input) if user_input else default
        except ValueError:
            print(f"⚠️ 无效输入，使用默认值: {default}")
            return default
    
    def get_float_input(self, prompt: str, default: float = 0.0) -> float:
        """获取浮点数输入"""
        try:
            user_input = self.get_user_input(f"{prompt} (默认: {default}): ")
            return float(user_input) if user_input else default
        except ValueError:
            print(f"⚠️ 无效输入，使用默认值: {default}")
            return default
    
    def print_result(self, title: str, result: Dict):
        """打印测试结果"""
        print(f"\n📊 {title}")
        print("-" * 40)
        
        if 'error' in result:
            print(f"❌ 错误: {result['error']}")
        else:
            print("✅ 成功")
            for key, value in result.items():
                if isinstance(value, dict):
                    print(f"   {key}:")
                    for k, v in value.items():
                        print(f"     {k}: {v}")
                else:
                    print(f"   {key}: {value}")
    
    # ==================== 映射功能 ====================
    
    def start_mapping(self):
        """开始映射"""
        print("🗺️ 开始映射...")
        result = self.make_request('POST', '/api/mapping/start')
        self.print_result("开始映射", result)
    
    def end_mapping(self):
        """结束映射"""
        print("⏹️ 结束映射...")
        result = self.make_request('POST', '/api/mapping/end')
        self.print_result("结束映射", result)
    
    def autonomous_mapping(self):
        """自主映射"""
        print("🤖 开始自主映射...")
        data = {
            "timeout": 300,  # 5分钟超时
            "auto_stop": True
        }
        result = self.make_request('POST', '/robotic_control/navigation/autonomous_mapping', data)
        self.print_result("自主映射", result)
    
    # ==================== 导航功能 ====================
    
    def start_navigation(self):
        """开始导航"""
        print("🚀 开始导航...")
        result = self.make_request('POST', '/api/navigation/start_loop')
        self.print_result("开始导航", result)
    
    def pause_navigation(self):
        """暂停导航"""
        print("⏸️ 暂停导航...")
        result = self.make_request('POST', '/api/navigation/pause')
        self.print_result("暂停导航", result)
    
    def recover_navigation(self):
        """恢复导航"""
        print("🔄 恢复导航...")
        result = self.make_request('POST', '/api/navigation/recover')
        self.print_result("恢复导航", result)
    
    def go_home(self):
        """返回原点"""
        print("🏠 返回原点...")
        result = self.make_request('POST', '/robotic_control/navigation/go_home')
        self.print_result("返回原点", result)
    
    def fixed_point_navigation(self):
        """定点导航"""
        print("📍 定点导航")
        print("-" * 20)
        
        # 选择导航方式
        nav_type = self.get_user_input("导航方式: 1-坐标导航, 2-节点导航 (默认: 1): ").strip()
        
        if nav_type == "2":
            # 节点导航
            node_id = self.get_int_input("目标节点ID")
            data = {
                "goal_node_id": node_id
            }
        else:
            # 坐标导航
            x = self.get_float_input("X 坐标")
            y = self.get_float_input("Y 坐标")
            yaw = self.get_float_input("Yaw 角度 (弧度)", 0.0)
            
            data = {
                "goal_coordinates": {
                    "x": x,
                    "y": y,
                    "yaw": yaw
                }
            }
        
        result = self.make_request('POST', '/robotic_control/navigation/fixed_point_nav', data)
        self.print_result("定点导航", result)
    
    def set_auto_nav(self):
        """设置自动导航"""
        print("🤖 设置自动导航")
        print("-" * 20)
        
        map_name = self.get_user_input("地图名称 (默认: default): ").strip() or "default"
        area_input = self.get_user_input("导航区域 [x1,y1,x2,y2] (可选): ").strip()
        path_input = self.get_user_input("路径点 [[x1,y1],[x2,y2],...] (可选): ").strip()
        
        data = {
            "map": map_name
        }
        
        if area_input:
            try:
                area = [float(x.strip()) for x in area_input.strip('[]').split(',')]
                data["area"] = area
            except:
                print("⚠️ 区域格式错误，跳过")
        
        if path_input:
            try:
                # 解析路径点
                path_str = path_input.strip('[]')
                path_points = []
                for point_str in path_str.split('],['):
                    point_str = point_str.strip('[]')
                    coords = [float(x.strip()) for x in point_str.split(',')]
                    path_points.append(coords)
                data["path_point"] = path_points
            except:
                print("⚠️ 路径点格式错误，跳过")
        
        result = self.make_request('POST', '/robotic_control/navigation/set_auto_nav', data)
        self.print_result("设置自动导航", result)
    
    # ==================== 节点和边管理 ====================
    
    def add_node_at_current_pose(self):
        """在当前位姿添加节点"""
        print("📍 在当前位姿添加节点...")
        result = self.make_request('POST', '/api/nodes/add_current')
        self.print_result("添加节点", result)
    
    def add_node_manual(self):
        """手动添加节点"""
        print("📍 手动添加节点")
        print("-" * 20)
        
        node_name = self.get_int_input("节点名称/ID")
        x = self.get_float_input("X 坐标")
        y = self.get_float_input("Y 坐标")
        z = self.get_float_input("Z 坐标", 0.0)
        yaw = self.get_float_input("Yaw 角度 (弧度)", 1.57)
        
        data = {
            "node_name": node_name,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw
        }
        
        result = self.make_request('POST', '/api/nodes/add', data)
        self.print_result("手动添加节点", result)
    
    def add_edge(self):
        """添加边"""
        print("🔗 添加边")
        print("-" * 20)
        
        edge_name = self.get_int_input("边名称/ID")
        start_node = self.get_int_input("起始节点ID")
        end_node = self.get_int_input("目标节点ID")
        dog_speed = self.get_float_input("机器狗速度", 1.0)
        
        data = {
            "edge_name": edge_name,
            "start_node": start_node,
            "end_node": end_node,
            "dog_speed": dog_speed
        }
        
        result = self.make_request('POST', '/api/edges/add', data)
        self.print_result("添加边", result)
    
    def delete_nodes(self):
        """删除节点"""
        print("🗑️ 删除节点")
        print("-" * 20)
        
        node_ids_input = self.get_user_input("节点ID列表 (用逗号分隔): ")
        try:
            node_ids = [int(x.strip()) for x in node_ids_input.split(',')]
            data = {"node_ids": node_ids}
            result = self.make_request('DELETE', '/api/nodes/delete', data)
            self.print_result("删除节点", result)
        except ValueError:
            print("❌ 节点ID格式错误")
    
    def delete_edges(self):
        """删除边"""
        print("🗑️ 删除边")
        print("-" * 20)
        
        edge_ids_input = self.get_user_input("边ID列表 (用逗号分隔): ")
        try:
            edge_ids = [int(x.strip()) for x in edge_ids_input.split(',')]
            data = {"edge_ids": edge_ids}
            result = self.make_request('DELETE', '/api/edges/delete', data)
            self.print_result("删除边", result)
        except ValueError:
            print("❌ 边ID格式错误")
    
    # ==================== 节点和边管理扩展功能 ====================
    
    def delete_all_nodes(self):
        """删除所有节点"""
        print("🗑️ 删除所有节点...")
        result = self.make_request('DELETE', '/api/nodes/delete_all')
        self.print_result("删除所有节点", result)
    
    def delete_all_edges(self):
        """删除所有边"""
        print("🗑️ 删除所有边...")
        result = self.make_request('DELETE', '/api/edges/delete_all')
        self.print_result("删除所有边", result)
    
    def query_nodes(self):
        """查询节点"""
        print("🔍 查询节点")
        print("-" * 20)
        attribute = self.get_int_input("查询属性 (默认: 1)", 1)
        data = {"attribute": attribute}
        result = self.make_request('POST', '/api/nodes/query', data)
        self.print_result("查询节点", result)
    
    def query_edges(self):
        """查询边"""
        print("🔍 查询边")
        print("-" * 20)
        attribute = self.get_int_input("查询属性 (默认: 2)", 2)
        data = {"attribute": attribute}
        result = self.make_request('POST', '/api/edges/query', data)
        self.print_result("查询边", result)
    
    def network_overview(self):
        """网络概览分析"""
        print("🌐 网络概览分析...")
        result = self.make_request('GET', '/api/network/overview')
        self.print_result("网络概览分析", result)
    
    def path_analysis(self):
        """路径分析"""
        print("🛤️ 路径分析")
        print("-" * 20)
        start_node = self.get_int_input("起始节点ID")
        end_node = self.get_int_input("目标节点ID")
        data = {
            "start_node": start_node,
            "end_node": end_node
        }
        result = self.make_request('POST', '/api/path/analysis', data)
        self.print_result("路径分析", result)
    
    # ==================== 内部存储管理 ====================
    
    def publish_all_nodes(self):
        """发布所有内部存储的节点"""
        print("📤 发布所有内部节点...")
        result = self.make_request('POST', '/api/internal/nodes/publish')
        self.print_result("发布所有内部节点", result)
    
    def publish_all_edges(self):
        """发布所有内部存储的边"""
        print("📤 发布所有内部边...")
        result = self.make_request('POST', '/api/internal/edges/publish')
        self.print_result("发布所有内部边", result)
    
    def publish_all_nodes_and_edges(self):
        """发布所有内部存储的节点和边"""
        print("📤 发布所有内部节点和边...")
        result = self.make_request('POST', '/api/internal/publish_all')
        self.print_result("发布所有内部节点和边", result)
    
    def clear_internal_storage(self):
        """清除内部存储"""
        print("🗑️ 清除内部存储...")
        result = self.make_request('POST', '/api/internal/clear')
        self.print_result("清除内部存储", result)
    
    def get_internal_storage_info(self):
        """获取内部存储信息"""
        print("📊 获取内部存储信息...")
        result = self.make_request('GET', '/api/internal/info')
        self.print_result("内部存储信息", result)
    
    # ==================== 自动收集功能 ====================
    
    def auto_collect_node(self):
        """自动收集节点"""
        print("📍 自动收集节点")
        print("-" * 20)
        auto_connect = self.get_user_input("自动连接 (y/n, 默认: y): ").lower().strip() != 'n'
        data = {"auto_connect": auto_connect}
        result = self.make_request('POST', '/api/auto_collect/node', data)
        self.print_result("自动收集节点", result)
    
    def collect_and_save_nodes_edges(self):
        """收集并保存节点和边"""
        print("💾 收集并保存节点和边")
        print("-" * 20)
        clear_after_save = self.get_user_input("保存后清除 (y/n, 默认: y): ").lower().strip() != 'n'
        data = {"clear_after_save": clear_after_save}
        result = self.make_request('POST', '/api/auto_collect/save', data)
        self.print_result("收集并保存节点和边", result)
    
    def auto_collect_loop(self):
        """自动收集循环"""
        print("🔄 自动收集循环")
        print("-" * 20)
        node_interval = self.get_float_input("节点间隔 (秒)", 2.0)
        max_nodes = self.get_int_input("最大节点数", 10)
        data = {
            "node_interval": node_interval,
            "max_nodes": max_nodes
        }
        result = self.make_request('POST', '/api/auto_collect/loop', data)
        self.print_result("自动收集循环", result)
    
    def prepare_for_collection(self):
        """准备收集"""
        print("🔄 准备收集...")
        result = self.make_request('POST', '/api/auto_collect/prepare')
        self.print_result("准备收集", result)
    
    def clear_and_start_mapping(self):
        """清除并开始映射"""
        print("🗑️ 清除并开始映射...")
        result = self.make_request('POST', '/api/auto_collect/clear_and_map')
        self.print_result("清除并开始映射", result)
    
    def get_collection_status(self):
        """获取收集状态"""
        print("📊 获取收集状态...")
        result = self.make_request('GET', '/api/auto_collect/status')
        self.print_result("收集状态", result)
    
    # ==================== Notice 缓存控制 ====================
    
    def configure_notice_cache(self):
        """配置 notice 缓存"""
        print("⚙️ 配置 Notice 缓存")
        print("-" * 20)
        cache_duration = self.get_float_input("缓存时间 (秒)", 2.0)
        auto_cache = self.get_user_input("自动缓存 (y/n, 默认: y): ").lower().strip() != 'n'
        data = {
            "cache_duration": cache_duration,
            "auto_cache": auto_cache
        }
        result = self.make_request('POST', '/api/notice_cache/configure', data)
        self.print_result("配置 Notice 缓存", result)
    
    def get_notice_cache_status(self):
        """获取 notice 缓存状态"""
        print("📊 获取 Notice 缓存状态...")
        result = self.make_request('GET', '/api/notice_cache/status')
        self.print_result("Notice 缓存状态", result)
    
    def test_notice_cache(self):
        """测试 notice 缓存"""
        print("🧪 测试 Notice 缓存")
        print("-" * 20)
        duration = self.get_float_input("测试时间 (秒)", 3.0)
        data = {"duration": duration}
        result = self.make_request('POST', '/api/notice_cache/test', data)
        self.print_result("测试 Notice 缓存", result)
    
    # ==================== 其他功能 ====================
    
    def close_all_nodes(self):
        """关闭所有节点"""
        print("🔒 关闭所有节点...")
        result = self.make_request('POST', '/api/nodes/close_all')
        self.print_result("关闭所有节点", result)
    
    def get_cloud_info(self):
        """获取云信息"""
        print("☁️ 获取云信息...")
        result = self.make_request('GET', '/api/cloud/info')
        self.print_result("云信息", result)
    
    def set_downsample_parameters(self):
        """设置下采样参数"""
        print("⚙️ 设置下采样参数")
        print("-" * 20)
        max_size = self.get_int_input("最大点云大小", 100000)
        voxel_size = self.get_float_input("体素大小", 0.05)
        data = {
            "max_size": max_size,
            "voxel_size": voxel_size
        }
        result = self.make_request('POST', '/api/downsample/configure', data)
        self.print_result("设置下采样参数", result)
    
    # ==================== 位姿管理 ====================
    
    def initialize_pose(self):
        """初始化位姿"""
        print("🎯 初始化位姿...")
        result = self.make_request('POST', '/api/pose/init')
        self.print_result("初始化位姿", result)
    
    def start_relocation(self):
        """开始重定位"""
        print("🔍 开始重定位...")
        result = self.make_request('POST', '/api/pose/relocation')
        self.print_result("开始重定位", result)
    
    def get_current_pose(self):
        """获取当前位姿"""
        print("📍 获取当前位姿...")
        result = self.make_request('GET', '/api/pose/current')
        self.current_pose = result
        self.print_result("当前位姿", result)
    
    def get_realtime_pose(self):
        """获取实时位姿"""
        print("📍 获取实时位姿...")
        result = self.make_request('GET', '/api/pose/realtime')
        self.print_result("实时位姿", result)
    
    def get_nav_state(self):
        """获取导航状态"""
        print("📊 获取导航状态...")
        result = self.make_request('GET', '/robotic_control/navigation/get_nav_state')
        self.nav_state = result
        self.print_result("导航状态", result)
    
    # ==================== 点云管理 ====================
    
    def clear_environment_cloud(self):
        """清除环境点云"""
        print("🗑️ 清除环境点云...")
        result = self.make_request('POST', '/api/pointcloud/clear/environment')
        self.print_result("清除环境点云", result)
    
    def clear_trajectory_cloud(self):
        """清除轨迹点云"""
        print("🗑️ 清除轨迹点云...")
        result = self.make_request('POST', '/api/pointcloud/clear/trajectory')
        self.print_result("清除轨迹点云", result)
    
    def clear_all_clouds(self):
        """清除所有点云"""
        print("🗑️ 清除所有点云...")
        result = self.make_request('POST', '/api/pointcloud/clear/all')
        self.print_result("清除所有点云", result)
    
    def save_environment_cloud(self):
        """保存环境点云"""
        print("💾 保存环境点云...")
        filename = self.get_user_input("文件名 (默认: environment_cloud.pcd): ").strip() or "environment_cloud.pcd"
        data = {"filename": filename}
        result = self.make_request('POST', '/api/pointcloud/save/environment', data)
        self.print_result("保存环境点云", result)
    
    def save_trajectory_cloud(self):
        """保存轨迹点云"""
        print("💾 保存轨迹点云...")
        filename = self.get_user_input("文件名 (默认: trajectory_cloud.pcd): ").strip() or "trajectory_cloud.pcd"
        data = {"filename": filename}
        result = self.make_request('POST', '/api/pointcloud/save/trajectory', data)
        self.print_result("保存轨迹点云", result)
    
    def save_combined_cloud(self):
        """保存组合点云"""
        print("💾 保存组合点云...")
        filename = self.get_user_input("文件名 (默认: combined_cloud.pcd): ").strip() or "combined_cloud.pcd"
        data = {"filename": filename}
        result = self.make_request('POST', '/api/pointcloud/save/combined', data)
        self.print_result("保存组合点云", result)
    
    # ==================== 可视化控制 ====================
    
    def start_visualization(self):
        """开始可视化"""
        print("🎬 开始可视化...")
        result = self.make_request('POST', '/api/visualization/start')
        self.print_result("开始可视化", result)
    
    def stop_visualization(self):
        """停止可视化"""
        print("⏹️ 停止可视化...")
        result = self.make_request('POST', '/api/visualization/stop')
        self.print_result("停止可视化", result)
    
    def check_visualization_status(self):
        """检查可视化状态"""
        print("📊 检查可视化状态...")
        result = self.make_request('GET', '/api/visualization/status')
        self.print_result("可视化状态", result)
    
    # ==================== 相机控制 ====================
    
    def get_camera_data(self):
        """获取相机数据"""
        print("📷 获取相机数据")
        print("-" * 20)
        
        camera_names_input = self.get_user_input("相机名称 (front,back 用逗号分隔): ")
        camera_names = [name.strip() for name in camera_names_input.split(',')] if camera_names_input else ["front"]
        
        camera_mode_input = self.get_user_input("相机模式 (RGB,DEPTH 用逗号分隔，可选): ")
        camera_mode = [mode.strip() for mode in camera_mode_input.split(',')] if camera_mode_input else None
        
        data = {
            "camera_name": camera_names
        }
        
        if camera_mode:
            data["camera_mode"] = camera_mode
        
        result = self.make_request('POST', '/api/camera/data', data)
        self.print_result("相机数据", result)
    
    # ==================== 系统状态 ====================
    
    def get_system_status(self):
        """获取系统状态"""
        print("📊 获取系统状态...")
        result = self.make_request('GET', '/api/status')
        self.print_result("系统状态", result)
    
    def health_check(self):
        """健康检查"""
        print("❤️ 健康检查...")
        result = self.make_request('GET', '/api/health')
        self.print_result("健康检查", result)
    
    def get_api_help(self):
        """获取API帮助"""
        print("📖 获取API帮助...")
        result = self.make_request('GET', '/api/help')
        self.print_result("API帮助", result)
    
    # ==================== 显示帮助 ====================
    
    def show_help(self):
        """显示帮助信息"""
        print("\n🎮 Interactive HTTP Navigator Agent 测试器")
        print("=" * 60)
        print("📋 可用命令:")
        print("\n🗺️ 映射功能:")
        print("  [M] 开始/结束映射")
        print("  [AM] 自主映射")
        print("\n🚀 导航功能:")
        print("  [S] 开始导航")
        print("  [P] 暂停导航")
        print("  [R] 恢复导航")
        print("  [H] 返回原点")
        print("  [FP] 定点导航")
        print("  [AN] 设置自动导航")
        print("  [NS] 获取导航状态")
        print("\n📍 位姿管理:")
        print("  [I] 初始化位姿")
        print("  [L] 开始重定位")
        print("  [G] 获取当前位姿")
        print("  [RT] 获取实时位姿")
        print("\n🗑️ 节点和边管理:")
        print("  [N] 在当前位姿添加节点")
        print("  [NM] 手动添加节点")
        print("  [E] 添加边")
        print("  [DN] 删除节点")
        print("  [DE] 删除边")
        print("  [DAN] 删除所有节点")
        print("  [DAE] 删除所有边")
        print("  [QN] 查询节点")
        print("  [QE] 查询边")
        print("  [NO] 网络概览分析")
        print("  [PA] 路径分析")
        print("  [CN] 关闭所有节点")
        print("\n🗑️ 点云管理:")
        print("  [C] 清除环境点云")
        print("  [T] 清除轨迹点云")
        print("  [A] 清除所有点云")
        print("  [V] 保存环境点云")
        print("  [R] 保存轨迹点云")
        print("  [S] 保存组合点云")
        print("\n💾 内部存储管理:")
        print("  [PN] 发布所有内部节点")
        print("  [PE] 发布所有内部边")
        print("  [PNE] 发布所有内部节点和边")
        print("  [CI] 清除内部存储")
        print("  [GI] 获取内部存储信息")
        print("\n🔄 自动收集功能:")
        print("  [AC] 自动收集节点")
        print("  [CS] 收集并保存节点和边")
        print("  [AL] 自动收集循环")
        print("  [PC] 准备收集")
        print("  [CM] 清除并开始映射")
        print("  [GS] 获取收集状态")
        print("\n📢 Notice 缓存控制:")
        print("  [NC] 配置 Notice 缓存")
        print("  [NS] 获取 Notice 缓存状态")
        print("  [NT] 测试 Notice 缓存")
        print("\n🎬 可视化控制:")
        print("  [VS] 开始可视化")
        print("  [VT] 停止可视化")
        print("  [VH] 检查可视化状态")
        print("\n📷 相机控制:")
        print("  [CA] 获取相机数据")
        print("\n📊 系统状态:")
        print("  [ST] 获取系统状态")
        print("  [HC] 健康检查")
        print("  [AH] API帮助")
        print("  [I] 获取云信息")
        print("  [D] 设置下采样参数")
        print("\n❓ 帮助 & 退出:")
        print("  [H] 显示帮助")
        print("  [Q] 退出")
        print("=" * 60)
    
    # ==================== 命令处理 ====================
    
    def process_command(self, command: str):
        """处理用户命令"""
        command = command.upper().strip()
        
        if command == 'M':
            # 切换映射
            choice = self.get_user_input("开始 (S) 或结束 (E) 映射? ").upper().strip()
            if choice == 'S':
                self.start_mapping()
            elif choice == 'E':
                self.end_mapping()
            else:
                print("❌ 无效选择")
        
        elif command == 'AM':
            self.autonomous_mapping()
        
        elif command == 'S':
            self.start_navigation()
        
        elif command == 'P':
            self.pause_navigation()
        
        elif command == 'R':
            self.recover_navigation()
        
        elif command == 'H':
            self.go_home()
        
        elif command == 'FP':
            self.fixed_point_navigation()
        
        elif command == 'AN':
            self.set_auto_nav()
        
        elif command == 'NS':
            self.get_nav_state()
        
        elif command == 'I':
            self.initialize_pose()
        
        elif command == 'L':
            self.start_relocation()
        
        elif command == 'G':
            self.get_current_pose()
        
        elif command == 'RT':
            self.get_realtime_pose()
        
        elif command == 'N':
            self.add_node_at_current_pose()
        
        elif command == 'NM':
            self.add_node_manual()
        
        elif command == 'E':
            self.add_edge()
        
        elif command == 'DN':
            self.delete_nodes()
        
        elif command == 'DE':
            self.delete_edges()
        
        elif command == 'DAN':
            self.delete_all_nodes()
        
        elif command == 'DAE':
            self.delete_all_edges()
        
        elif command == 'QN':
            self.query_nodes()
        
        elif command == 'QE':
            self.query_edges()
        
        elif command == 'NO':
            self.network_overview()
        
        elif command == 'PA':
            self.path_analysis()
        
        elif command == 'PN':
            self.publish_all_nodes()
        
        elif command == 'PE':
            self.publish_all_edges()
        
        elif command == 'PNE':
            self.publish_all_nodes_and_edges()
        
        elif command == 'CI':
            self.clear_internal_storage()
        
        elif command == 'GI':
            self.get_internal_storage_info()
        
        elif command == 'AC':
            self.auto_collect_node()
        
        elif command == 'CS':
            self.collect_and_save_nodes_edges()
        
        elif command == 'AL':
            self.auto_collect_loop()
        
        elif command == 'PC':
            self.prepare_for_collection()
        
        elif command == 'CM':
            self.clear_and_start_mapping()
        
        elif command == 'GS':
            self.get_collection_status()
        
        elif command == 'NC':
            self.configure_notice_cache()
        
        elif command == 'NS':
            self.get_notice_cache_status()
        
        elif command == 'NT':
            self.test_notice_cache()
        
        elif command == 'CN':
            self.close_all_nodes()
        
        elif command == 'I':
            self.get_cloud_info()
        
        elif command == 'D':
            self.set_downsample_parameters()
        
        elif command == 'C':
            self.clear_environment_cloud()
        
        elif command == 'T':
            self.clear_trajectory_cloud()
        
        elif command == 'A':
            self.clear_all_clouds()
        
        elif command == 'V':
            self.save_environment_cloud()
        
        elif command == 'R':
            self.save_trajectory_cloud()
        
        elif command == 'S':
            self.save_combined_cloud()
        
        elif command == 'VS':
            self.start_visualization()
        
        elif command == 'VT':
            self.stop_visualization()
        
        elif command == 'VH':
            self.check_visualization_status()
        
        elif command == 'CA':
            self.get_camera_data()
        
        elif command == 'ST':
            self.get_system_status()
        
        elif command == 'HC':
            self.health_check()
        
        elif command == 'AH':
            self.get_api_help()
        
        elif command == 'H':
            self.show_help()
        
        elif command == 'Q':
            print("👋 退出测试器")
            return False
        
        else:
            print(f"❌ 未知命令: {command}")
            print("💡 输入 'H' 查看帮助")
        
        return True
    
    def run(self):
        """运行交互式测试器"""
        print("🚀 启动交互式 HTTP 测试器...")
        self.show_help()
        
        while True:
            try:
                command = self.get_user_input("\n🎮 请输入命令: ")
                if not self.process_command(command):
                    break
            except KeyboardInterrupt:
                print("\n👋 用户中断，退出测试器")
                break
            except Exception as e:
                print(f"❌ 处理命令时出错: {e}")

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Interactive HTTP Navigator Agent Tester")
    parser.add_argument("--host", default="localhost", help="HTTP 服务器主机地址")
    parser.add_argument("--port", type=int, default=8080, help="HTTP 服务器端口")
    
    args = parser.parse_args()
    
    base_url = f"http://{args.host}:{args.port}"
    
    # 检查服务器连接
    try:
        response = requests.get(f"{base_url}/api/health", timeout=5)
        if response.status_code == 200:
            print(f"✅ 服务器连接成功: {base_url}")
        else:
            print(f"⚠️ 服务器响应异常: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"❌ 无法连接到服务器 {base_url}: {e}")
        print("💡 请确保 http_navigator_agent.py 正在运行")
        return
    
    # 启动测试器
    tester = InteractiveHTTPTester(base_url)
    tester.run()

if __name__ == "__main__":
    main() 
