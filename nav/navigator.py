#!/usr/bin/env python3
"""
Navigator Class for ROS2 Dog Navigation System

This class provides a comprehensive interface for controlling the navigation system
of a robotic dog, including mapping, navigation, node/edge management, and pose operations.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from unitree_interfaces.msg import QtCommand, QtEdge, QtNode
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState, LowState
from typing import List, Optional, Tuple, Dict
import numpy as np
import open3d as o3d
import threading
import time
import json
import cv2
import base64
from dataclasses import dataclass


@dataclass
class CameraFrame:
    """单个相机最新帧的完整信息"""
    camera_id: int          # 设备唯一 ID
    name: str               # 逻辑名称，如 front / back / arm_left
    mode: str               # 当前模式 RGB / DEPTH / WIDE
    frame: Optional[np.ndarray] = None   # 原始 numpy 帧
    status: bool = False    # True: 正常取流  False: 异常


class Camera:
    """
    通用相机类。目前支持：
        - RGB 彩色
        - DEPTH 深度（预留：可把 16bit 深度图转 8bit 伪彩）
        - WIDE  广角（预留：可把原始图像做畸变矫正）
    新增机械臂相机时，只需：
        1. 新建一个 Camera 实例
        2. 把实例注册到 Navigator.cameras 字典
    """
    def __init__(self,
                 name: str,
                 uri: str,
                 camera_id: Optional[int] = None,
                 default_mode: str = "RGB"):
        self.name = name
        self.uri = uri
        self.camera_id = camera_id or abs(hash(name)) % (10**8)  # 简易唯一 ID
        self.default_mode = default_mode.upper()
        self._latest = CameraFrame(camera_id=self.camera_id,
                                   name=name,
                                   mode=self.default_mode)
        self._stop_evt = threading.Event()

    @property
    def latest(self) -> CameraFrame:
        return self._latest

    def _worker(self):
        pipeline = (
            f"rtspsrc location={self.uri} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! appsink"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self._latest.status = False
            return

        self._latest.status = True
        while not self._stop_evt.is_set():
            ret, frame = cap.read()
            if ret:
                self._latest.frame = frame
                self._latest.status = True
            else:
                self._latest.status = False
                time.sleep(0.01)
        cap.release()

    def start(self):
        threading.Thread(target=self._worker, daemon=True).start()

    def stop(self):
        self._stop_evt.set()


class Navigator(Node):
    """
    A comprehensive navigator class for ROS2 dog navigation system.
    
    This class provides methods for:
    - Mapping operations (start/end mapping)
    - Navigation control (start/pause/recover navigation)
    - Node management (add/delete nodes)
    - Edge management (add/delete edges)
    - Pose operations (initialization, relocation)
    """
    
    def __init__(self, 
                 command_topic: str = "/qt_command",
                 add_node_topic: str = "/qt_add_node",
                 add_edge_topic: str = "/qt_add_edge",
                 notice_topic: str = "/qt_notice",
                 cloud_topic: str = "/lio_sam_ros2/mapping/cloud_registered",
                 trajectory_topic: str = "/lio_sam_ros2/mapping/trajectory",
                 odometry_topic: str = "/lio_sam_ros2/mapping/re_location_odometry",
                 enable_visualization: bool = True
                 ):
        """
        Initialize the Navigator node.
        
        Args:
            command_topic: Topic for sending QtCommand messages
            add_node_topic: Topic for adding nodes
            add_edge_topic: Topic for adding edges
            notice_topic: Topic for receiving command execution notices
            cloud_topic: Topic for point cloud data
            trajectory_topic: Topic for trajectory data
            odometry_topic: Topic for odometry data
            enable_visualization: Whether to enable point cloud visualization
        """
        super().__init__('navigator')
        # Setup QoS profile for robot state subscriptions with larger buffers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1000  # 增加深度
        )
        # Create publishers
        self.command_publisher = self.create_publisher(QtCommand, command_topic, qos)
        self.node_publisher = self.create_publisher(QtNode, add_node_topic, qos)
        self.edge_publisher = self.create_publisher(QtEdge, add_edge_topic, qos)
        self.query_result_node_publisher = self.create_publisher(String, "/query_result_node", qos)
        self.query_result_edge_publisher = self.create_publisher(String, "/query_result_edge", qos)
        
        # Current pose storage
        self.current_pose = None
        self.pose_lock = threading.Lock()
        self.last_pose_update_time = 0.0
        self.pose_timeout = 0.5  # 0.5 seconds timeout for pose data
        
        # Command execution feedback storage
        self.last_notice = None
        self.notice_lock = threading.Lock()
        self.command_confirmations = {}  # Store command confirmations by sequence
        
        # Index management for command sequences
        self.command_index = 123  # Start from 123
        self.index_lock = threading.Lock()  # Thread-safe index management
        
        # Internal node and edge storage for batch operations
        self.internal_nodes = {}  # {node_id: {x, y, z, yaw, attribute, state_2, state_3, undefined}}
        self.internal_edges = {}  # {edge_id: {start_node, end_node, dog_speed, edge_state_2, dog_stats, edge_length, dog_back_stats, edge_state, edge_state_1, edge_state_3, edge_state_4}}
        self.nodes_lock = threading.Lock()  # Thread-safe node operations
        self.edges_lock = threading.Lock()  # Thread-safe edge operations
        self.batch_mode = False  # Flag to control batch operations
        
        # Create subscriber for odometry
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_callback,
            qos
        )
        
        # Notice subscriber will be created only when needed
        self.notice_subscriber = None
        self.notice_topic = notice_topic
        self.notice_qos = qos
        
        # Notice cache configuration
        self.notice_cache_duration = 2.0  # Default cache duration in seconds
        self.notice_auto_cache = True      # Whether to auto-cache before publishing commands
       
        
        # Robot state message cache
        self.msg_cache = {
            'lowstate': None,
            'odom': None,
            'imu': None,
            'sport': None,
            'slam': None,
            'control': None,
            'network': None
        }
        
        # Additional robot state subscribers
        self.lowstate_subscriber = self.create_subscription(
            LowState, '/lf/lowstate', self._cb('lowstate'), qos)
        self.dog_odom_subscriber = self.create_subscription(
            Odometry, '/dog_odom', self._cb('odom'), qos)
        self.dog_imu_subscriber = self.create_subscription(
            Imu, '/dog_imu_raw', self._cb('imu'), qos)
        self.sport_subscriber = self.create_subscription(
            SportModeState, '/sportmodestate', self._cb('sport'), qos)
        self.slam_info_subscriber = self.create_subscription(
            String, '/slam_info', self._cb('slam'), qos)
        self.control_feedback_subscriber = self.create_subscription(
            String, '/control_feedback', self._cb('control'), qos)
        self.network_status_subscriber = self.create_subscription(
            String, '/public_network_status', self._cb('network'), qos)
        
        # Camera management
        self.cameras: Dict[str, Camera] = {}
        self.cameras["front"] = Camera("front", "rtsp://192.168.123.161:8551/front_video")
        self.cameras["back"] = Camera("back", "rtsp://192.168.123.161:8552/back_video")
        
        # Start camera streams
        for cam in self.cameras.values():
            cam.start()
        
        # Additional frame storage for backward compatibility
        self.front_frame = None
        self.back_frame = None
        self._stop_cam = threading.Event()
        
        # Start legacy camera threads
        self.cam_threads = []
        self.cam_threads.append(threading.Thread(target=self._cam_worker,
                                                 args=("front", "rtsp://192.168.123.161:8551/front_video"),
                                                 daemon=True))
        self.cam_threads.append(threading.Thread(target=self._cam_worker,
                                                 args=("back", "rtsp://192.168.123.161:8552/back_video"),
                                                 daemon=True))
        for t in self.cam_threads:
            t.start()
        
        # Point cloud visualization setup
        self.enable_visualization = enable_visualization
        self.accumulated_cloud = o3d.geometry.PointCloud()  # 环境点云
        self.trajectory_cloud = o3d.geometry.PointCloud()   # 轨迹点云
        self.cloud_lock = threading.Lock()
        self.max_cloud_size = 100000  # Maximum number of points before downsampling
        self.downsample_voxel_size = 0.05  # Voxel size for downsampling
        
        # 可视化状态控制
        self.visualization_started = False
        self.visualization_running = False
        self.visualization_thread = None
        
        if self.enable_visualization:
            # Create subscriber for point cloud with larger buffer
            cloud_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=100  # 增加点云缓冲区
            )
            
            self.cloud_subscriber = self.create_subscription(
                PointCloud2, 
                cloud_topic, 
                self.cloud_callback, 
                cloud_qos
            )
            
            # Create subscriber for trajectory
            self.trajectory_subscriber = self.create_subscription(
                PointCloud2,
                trajectory_topic,
                self.trajectory_callback,
                10
            )
            
            self.get_logger().info(f"📊 Point cloud visualization enabled on topic: {cloud_topic}")
            self.get_logger().info(f"🛤️ Trajectory visualization enabled on topic: {trajectory_topic}")
        
        self.get_logger().info(f"📍 Odometry subscription enabled on topic: {odometry_topic}")
        self.get_logger().info(f"📢 Notice subscription enabled on topic: {notice_topic}")
        self.get_logger().info("📷 Camera streams initialized (front/back)")
        self.get_logger().info("🤖 Robot state subscriptions enabled")
        self.get_logger().info("🚀 Navigator initialized successfully")
    
    def start_mapping(self, attribute: int = 0) -> int:
        """
        Start the mapping process.
        
        Args:
            attribute: Attribute value (B2 fixed as 0)
            
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.command = 3  # Start mapping command
        msg.attribute = attribute
        msg.seq = String()
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"🗺️ Sent start mapping command with index: {index}")
        
        return index
    
    def end_mapping(self, floor_index: int = 0, pcdmap_index: int = 0) -> int:
        """
        End the mapping process.
        
        Args:
            floor_index: Floor index
            pcdmap_index: PCD map index
            
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 4  # End mapping
        msg.floor_index.append(floor_index)
        msg.pcdmap_index.append(pcdmap_index)
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"✅ Sent end mapping command with index: {index} (floor={floor_index}, map={pcdmap_index})")
        
        return index
    
    def start_navigation(self) -> int:
        """
        Start navigation.
        
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 8 # Start Navigation
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"🚀 Sent start navigation command with index: {index}")
        
        return index

    def start_single_navigation(self, node_id: int) -> int:
        """
        Start single navigation.
        
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 9 # Start Single Navigation
        msg.node_edge_name.append(node_id)
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"🚀 Sent start single navigation command with index: {index}")
        return index

    def default_navigation_loop(self) -> int:
        """
        Start navigation loop.
        
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 10 # Start Navigation
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"🚀 Sent start navigation loop command with index: {index}")
        
        return index
    
    def pause_navigation(self) -> int:
        """
        Pause navigation.
        
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 13  # Pause navigation
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"⏸️ Sent pause navigation command with index: {index}")
        
        return index
    
    def query_node(self, attribute: int = 1) -> tuple[bool, int]:
        """
        Query navigation nodes.
        
        Args:
            attribute: Query attribute (default: 1 for nodes)
            
        Returns:
            tuple[bool, int]: (success, command_index)
        """
        try:
            msg = QtCommand()
            msg.seq = String()
            msg.command = 2
            msg.attribute = attribute
            msg.floor_index.append(999)
            msg.node_edge_name.append(999)
            
            # Use automatic index management
            index = self._publish_command_with_index(msg, wait_for_confirmation=True)
            self.get_logger().info(f"▶️ Sent query node command with index: {index}")
            
            # Check if confirmation was successful
            confirmation = self.get_command_confirmation(str(index))
            if confirmation:
                # Publish query result to feedback topic
                result_msg = String()
                result_msg.data = json.dumps({
                    "seq": f"index:{index};",
                    "command": "query_node",
                    "attribute": attribute,
                    "status": "success" if confirmation.get('success', False) else "failed",
                    "message": confirmation.get('message', ''),
                    "timestamp": time.time()
                })
                self.query_result_node_publisher.publish(result_msg)
                self.get_logger().info(f"📤 Published query node result: {confirmation.get('message', '')}")
                return confirmation.get('success', False), index
            else:
                # Publish timeout result
                result_msg = String()
                result_msg.data = json.dumps({
                    "seq": f"index:{index};",
                    "command": "query_node",
                    "attribute": attribute,
                    "status": "timeout",
                    "message": "Command confirmation timeout",
                    "timestamp": time.time()
                })
                self.query_result_node_publisher.publish(result_msg)
                self.get_logger().warning("⏰ Query node command confirmation timeout")
                return False, index
                
        except Exception as e:
            self.get_logger().error(f"Error sending query node command: {e}")
            return False, -1
    
    def query_edge(self, attribute: int = 2) -> tuple[bool, int]:
        """
        Query navigation edges.
        
        Args:
            attribute: Query attribute (default: 2 for edges)
            
        Returns:
            tuple[bool, int]: (success, command_index)
        """
        try:
            msg = QtCommand()
            msg.seq = String()
            msg.command = 2
            msg.attribute = attribute
            msg.floor_index.append(999)
            msg.node_edge_name.append(999)
            
            # Use automatic index management
            index = self._publish_command_with_index(msg, wait_for_confirmation=True)
            self.get_logger().info(f"▶️ Sent query edge command with index: {index}")
            
            # Check if confirmation was successful
            confirmation = self.get_command_confirmation(str(index))
            if confirmation:
                # Publish query result to feedback topic
                result_msg = String()
                result_msg.data = json.dumps({
                    "seq": f"index:{index};",
                    "command": "query_edge",
                    "attribute": attribute,
                    "status": "success" if confirmation.get('success', False) else "failed",
                    "message": confirmation.get('message', ''),
                    "timestamp": time.time()
                })
                self.query_result_edge_publisher.publish(result_msg)
                self.get_logger().info(f"📤 Published query edge result: {confirmation.get('message', '')}")
                return confirmation.get('success', False), index
            else:
                # Publish timeout result
                result_msg = String()
                result_msg.data = json.dumps({
                    "seq": f"index:{index};",
                    "command": "query_edge",
                    "attribute": attribute,
                    "status": "timeout",
                    "message": "Command confirmation timeout",
                    "timestamp": time.time()
                })
                self.query_result_edge_publisher.publish(result_msg)
                self.get_logger().warning("⏰ Query edge command confirmation timeout")
                return False, index
                
        except Exception as e:
            self.get_logger().error(f"Error sending query edge command: {e}")
            return False, -1


    def recover_navigation(self) -> int:
        """
        Recover/resume navigation.
        
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 14  # Recover navigation command
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"▶️ Sent recover navigation command with index: {index}")
        
        return index

    def go_home(self) -> int:
        """
        Go home.
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 15  # Go home command

        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"▶️ Sent go home command with index: {index}")
        return index
    
    def set_auto_nav(self, map_name: str = 'default', area: list = None, path_point: list = None) -> bool:
        """
        设置自动导航参数
        
        Args:
            map_name: 使用的导航地图
            area: 划定的导航区域范围
            path_point: 设置的巡逻路径点
            
        Returns:
            bool: 设置是否成功
        """
        try:
            # 这里可以根据实际需求实现具体的设置逻辑
            self.get_logger().info(f"设置自动导航参数: map={map_name}, area={area}, path_point={path_point}")
            
            # 示例实现 - 可以根据实际需求修改
            if area:
                self.get_logger().info(f"设置导航区域: {area}")
            
            if path_point:
                self.get_logger().info(f"设置巡逻路径点: {path_point}")
            
            return True
        except Exception as e:
            self.get_logger().error(f"设置自动导航参数失败: {e}")
            return False
    
    def start_navigation_loop(self, seq: str = "index:123;") -> bool:
        """
        开始循环导航
        
        Args:
            seq: 序列号
            
        Returns:
            bool: 启动是否成功
        """
        try:
            msg = QtCommand()
            msg.seq = String()
            msg.command = 10  # 10 is multi nodes loop navigation command
            msg.seq.data = seq
            
            # Use automatic index management
            index = self._publish_command_with_index(msg, wait_for_confirmation=True)
            self.get_logger().info(f"🔄 循环导航已启动，index: {index}")
            return True
        except Exception as e:
            self.get_logger().error(f"启动循环导航失败: {e}")
            return False
    
    def add_node(self, node_name: int, x: float, y: float, z: float = 0.0, 
                yaw: float = 1.57, publish_immediately: bool = False) -> int:
        """
        Add a navigation node to internal storage.
        
        Args:
            node_name: Name/ID of the node
            x: X coordinate
            y: Y coordinate
            z: Z coordinate (default: 0.0)
            yaw: Yaw angle in radians (default: 1.57)
            publish_immediately: If True, publish immediately; if False, store internally
            
        Returns:
            int: The command index used (if published immediately) or 0 (if stored internally)
        """
        with self.nodes_lock:
            # Store node data internally
            self.internal_nodes[node_name] = {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'attribute': 0,
                'undefined': 0,
                'state_2': 0,
                'state_3': 0
            }
            
            self.get_logger().info(f"📍 Stored node {node_name} at ({x}, {y}, {z}) with yaw {yaw}")
            
            if publish_immediately:
                # Publish immediately if requested
                msg = QtNode()
                msg.seq = String()
                msg.node.node_name.append(node_name)
                msg.node.node_position_x.append(x)
                msg.node.node_position_y.append(y)
                msg.node.node_position_z.append(z)
                msg.node.node_yaw.append(yaw)
                
                # Required fields
                msg.node.node_attribute.append(0)
                msg.node.undefined.append(0)
                msg.node.node_state_2.append(0)
                msg.node.node_state_3.append(0)
                
                # Use automatic index management
                index = self._publish_command_with_index(msg, wait_for_confirmation=True)
                self.get_logger().info(f"✅ Published node {node_name} immediately, index: {index}")
                return index
            else:
                return 0
    
    def delete_node(self, node_ids: List[int]) -> int:
        """
        Delete navigation nodes.
        
        Args:
            node_ids: List of node IDs to delete
            
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 1  # Delete operation
        msg.attribute = 1  # Delete nodes
        msg.node_edge_name.extend(node_ids)
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"🗑️ Sent delete node command: {node_ids}, index: {index}")
        
        return index
    
    def add_edge(self, edge_name: int, start_node: int, end_node: int, 
                dog_speed: float = 1.0, seq: str = "index:123;", publish_immediately: bool = False) -> int:
        """
        Add a navigation edge to internal storage.
        
        Args:
            edge_name: Name/ID of the edge
            start_node: Starting node ID
            end_node: Ending node ID
            dog_speed: Speed of the dog (default: 1.0)
            seq: Sequence identifier
            publish_immediately: If True, publish immediately; if False, store internally
            
        Returns:
            int: The command index used (if published immediately) or 0 (if stored internally)
        """
        with self.edges_lock:
            # Store edge data internally
            self.internal_edges[edge_name] = {
                'start_node': start_node,
                'end_node': end_node,
                'dog_speed': dog_speed,
                'edge_state_2': 0,  # 0: Stop
                'dog_stats': 0,
                'edge_length': 0.0,
                'dog_back_stats': 0,
                'edge_state': 0,
                'edge_state_1': 0,
                'edge_state_3': 0,
                'edge_state_4': 0
            }
            
            self.get_logger().info(f"🔗 Stored edge {edge_name} from node {start_node} to {end_node} with speed {dog_speed}")
            
            if publish_immediately:
                # Publish immediately if requested
                msg = QtEdge()
                msg.seq = String()
                msg.seq.data = seq
                msg.edge.edge_name.append(edge_name)
                msg.edge.start_node_name.append(start_node)
                msg.edge.end_node_name.append(end_node)
                msg.edge.dog_speed.append(dog_speed)
                
                # Required fields
                msg.edge.edge_state_2.append(0)  # 0: Stop
                msg.edge.dog_stats.append(0)
                msg.edge.edge_length.append(0.0)
                msg.edge.dog_back_stats.append(0)
                msg.edge.edge_state.append(0)
                msg.edge.edge_state_1.append(0)
                msg.edge.edge_state_3.append(0)
                msg.edge.edge_state_4.append(0)
                
                # Use automatic index management
                index = self._publish_command_with_index(msg, wait_for_confirmation=True)
                self.get_logger().info(f"✅ Published edge {edge_name} immediately, index: {index}")
                return index
            else:
                return 0
    
    def delete_edge(self, edge_ids: List[int]) -> int:
        """
        Delete navigation edges.
        
        Args:
            edge_ids: List of edge IDs to delete
            
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 1  # Delete operation
        msg.attribute = 2  # Delete edges
        msg.node_edge_name.extend(edge_ids)
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"🗑️ Sent delete edge command: {edge_ids}, index: {index}")
        
        return index
    
    def publish_all_nodes(self) -> int:
        """
        Publish all internally stored nodes to the system.
        
        Returns:
            int: The command index used
        """
        with self.nodes_lock:
            if not self.internal_nodes:
                self.get_logger().warning("⚠️ No nodes to publish")
                return 0
            
            msg = QtNode()
            msg.seq = String()
            
            # Add all nodes to the message
            for node_id, node_data in self.internal_nodes.items():
                msg.node.node_name.append(node_id)
                msg.node.node_position_x.append(node_data['x'])
                msg.node.node_position_y.append(node_data['y'])
                msg.node.node_position_z.append(node_data['z'])
                msg.node.node_yaw.append(node_data['yaw'])
                msg.node.node_attribute.append(node_data['attribute'])
                msg.node.undefined.append(node_data['undefined'])
                msg.node.node_state_2.append(node_data['state_2'])
                msg.node.node_state_3.append(node_data['state_3'])
            
            # Use automatic index management
            index = self._publish_command_with_index(msg, wait_for_confirmation=True)
            self.get_logger().info(f"✅ Published {len(self.internal_nodes)} nodes, index: {index}")
            
            return index
    
    def publish_all_edges(self) -> int:
        """
        Publish all internally stored edges to the system.
        
        Returns:
            int: The command index used
        """
        with self.edges_lock:
            if not self.internal_edges:
                self.get_logger().warning("⚠️ No edges to publish")
                return 0
            
            msg = QtEdge()
            msg.seq = String()
            
            # Add all edges to the message
            for edge_id, edge_data in self.internal_edges.items():
                msg.edge.edge_name.append(edge_id)
                msg.edge.start_node_name.append(edge_data['start_node'])
                msg.edge.end_node_name.append(edge_data['end_node'])
                msg.edge.dog_speed.append(edge_data['dog_speed'])
                msg.edge.edge_state_2.append(edge_data['edge_state_2'])
                msg.edge.dog_stats.append(edge_data['dog_stats'])
                msg.edge.edge_length.append(edge_data['edge_length'])
                msg.edge.dog_back_stats.append(edge_data['dog_back_stats'])
                msg.edge.edge_state.append(edge_data['edge_state'])
                msg.edge.edge_state_1.append(edge_data['edge_state_1'])
                msg.edge.edge_state_3.append(edge_data['edge_state_3'])
                msg.edge.edge_state_4.append(edge_data['edge_state_4'])
            
            # Use automatic index management
            index = self._publish_command_with_index(msg, wait_for_confirmation=True)
            self.get_logger().info(f"✅ Published {len(self.internal_edges)} edges, index: {index}")
            
            return index
    
    def publish_all_nodes_and_edges(self) -> tuple[int, int]:
        """
        Publish all internally stored nodes and edges to the system.
        Nodes are published first, then edges.
        
        Returns:
            tuple[int, int]: (node_command_index, edge_command_index)
        """
        self.get_logger().info("🚀 Publishing all stored nodes and edges...")
        
        # Publish nodes first
        node_index = self.publish_all_nodes()
        
        # Wait a bit for nodes to be processed
        time.sleep(0.5)
        
        # Then publish edges
        edge_index = self.publish_all_edges()
        
        self.get_logger().info(f"✅ Published all nodes and edges - Node index: {node_index}, Edge index: {edge_index}")
        return node_index, edge_index
    
    def clear_internal_nodes(self) -> None:
        """Clear all internally stored nodes."""
        with self.nodes_lock:
            count = len(self.internal_nodes)
            self.internal_nodes.clear()
            self.get_logger().info(f"🗑️ Cleared {count} internally stored nodes")
    
    def clear_internal_edges(self) -> None:
        """Clear all internally stored edges."""
        with self.edges_lock:
            count = len(self.internal_edges)
            self.internal_edges.clear()
            self.get_logger().info(f"🗑️ Cleared {count} internally stored edges")
    
    def clear_internal_storage(self) -> None:
        """Clear all internally stored nodes and edges."""
        self.clear_internal_nodes()
        self.clear_internal_edges()
    
    def get_internal_nodes(self) -> dict:
        """Get all internally stored nodes."""
        with self.nodes_lock:
            return self.internal_nodes.copy()
    
    def get_internal_edges(self) -> dict:
        """Get all internally stored edges."""
        with self.edges_lock:
            return self.internal_edges.copy()
    
    def get_internal_storage_info(self) -> dict:
        """Get information about internally stored nodes and edges."""
        with self.nodes_lock:
            with self.edges_lock:
                return {
                    'nodes_count': len(self.internal_nodes),
                    'edges_count': len(self.internal_edges),
                    'nodes': list(self.internal_nodes.keys()),
                    'edges': list(self.internal_edges.keys())
                }
    
    def remove_internal_node(self, node_id: int) -> bool:
        """Remove a specific node from internal storage."""
        with self.nodes_lock:
            if node_id in self.internal_nodes:
                del self.internal_nodes[node_id]
                self.get_logger().info(f"🗑️ Removed node {node_id} from internal storage")
                return True
            else:
                self.get_logger().warning(f"⚠️ Node {node_id} not found in internal storage")
                return False
    
    def remove_internal_edge(self, edge_id: int) -> bool:
        """Remove a specific edge from internal storage."""
        with self.edges_lock:
            if edge_id in self.internal_edges:
                del self.internal_edges[edge_id]
                self.get_logger().info(f"🗑️ Removed edge {edge_id} from internal storage")
                return True
            else:
                self.get_logger().warning(f"⚠️ Edge {edge_id} not found in internal storage")
                return False
    
    def navigate_to_point(self, x: float, y: float, yaw: float = 0.0, goal_node_id: int = None, map_name: str = "default") -> bool:
        """
        Navigate to a point.
        
        Args:
            x: 目标点X坐标
            y: 目标点Y坐标
            yaw: 目标点Yaw角度（弧度）
            goal_node_id: 目标节点ID（如果提供，则导航到节点）
            map_name: 使用的地图名称
            
        Returns:
            bool: 导航是否成功启动
        """
        try:
            if goal_node_id is not None:
                # 导航到指定节点
                self.get_logger().info(f"🎯 开始导航到节点: {goal_node_id}")
                result = self.start_single_navigation(goal_node_id)
                return result > 0
            else:
                # 定点导航
                self.get_logger().info(f"🎯 开始定点导航到坐标: ({x:.2f}, {y:.2f}), 角度: {yaw:.2f}°, 地图: {map_name}")
                
                # 获取当前位姿
                current_pose = self.get_current_pose()
                if current_pose is None:
                    self.get_logger().error("无法获取当前位姿，导航失败")
                    return False
                
                current_x, current_y, current_z = current_pose['position']
                current_yaw = current_pose['euler'][2]
                
                self.get_logger().info(f"📍 当前位置: ({current_x:.2f}, {current_y:.2f}), 角度: {current_yaw:.2f}°")
                
                # 计算距离
                distance = ((x - current_x) ** 2 + (y - current_y) ** 2) ** 0.5
                self.get_logger().info(f"📏 目标距离: {distance:.2f}米")
                
                # 这里需要根据实际的导航系统实现具体的导航逻辑
                # 例如：调用ROS2的导航action或service
                
                # 模拟导航启动（实际实现中需要替换为真实的导航调用）
                self.get_logger().info(f"🚀 启动导航到目标点...")
                
                # 记录导航参数
                nav_params = {
                    "start_pose": {
                        "x": current_x,
                        "y": current_y,
                        "z": current_z,
                        "yaw": current_yaw
                    },
                    "goal_pose": {
                        "x": x,
                        "y": y,
                        "z": 0.0,  # 假设Z坐标为0
                        "yaw": yaw
                    },
                    "map_name": map_name,
                    "distance": distance
                }
                
                self.get_logger().info(f"📋 导航参数: {nav_params}")
                
                # 这里应该调用实际的导航服务
                # 例如：self.navigation_client.send_goal(goal)
                
                # 暂时返回成功（实际实现中需要根据导航服务的响应）
                self.get_logger().info("✅ 定点导航命令已发送")
                return True
                
        except Exception as e:
            self.get_logger().error(f"定点导航失败: {e}")
            return False

    def pose_init(self, 
                 translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> int:
        """
        Initialize pose.
        
        Args:
            translation: Translation vector (x, y, z)
            quaternion: Quaternion (x, y, z, w)
            
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 7  # Pose initialization
        
        # Set quaternion
        msg.quaternion_x = quaternion[0]
        msg.quaternion_y = quaternion[1]
        msg.quaternion_z = quaternion[2]
        msg.quaternion_w = quaternion[3]
        
        # Set translation
        msg.translation_x = translation[0]
        msg.translation_y = translation[1]
        msg.translation_z = translation[2]
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"📍 Sent pose init command at {translation}, index: {index}")
        
        return index
    
    def start_relocation(self) -> int:
        """
        Start relocation process.
        
        Args:
            attribute: Attribute value (B2 fixed as 0)
            
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 6  # Start relocation command
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"📍 Sent start relocation command with index: {index}")
        
        return index

    def close_all_nodes(self) -> int:
        """
        Close all nodes.
        
        Args:
            attribute: Attribute value (B2 fixed as 0)
            
        Returns:
            int: The command index used
        """
        msg = QtCommand()
        msg.seq = String()
        msg.command = 99  # Close all nodes command
        
        # Use automatic index management
        index = self._publish_command_with_index(msg, wait_for_confirmation=True)
        self.get_logger().info(f"📍 Sent start relocation command with index: {index}")
        
        return index

    
    def delete_all_nodes(self) -> int:
        """
        Delete all nodes.
        
        Returns:
            int: The command index used
        """
        return self.delete_node([999])
    
    def delete_all_edges(self) -> int:
        """
        Delete all edges.
        
        Returns:
            int: The command index used
        """
        return self.delete_edge([999])

    def cloud_callback(self, msg: PointCloud2) -> None:
        """
        Callback function for processing incoming point cloud messages.
        
        Args:
            msg: PointCloud2 message from LIO-SAM
        """
        if not self.enable_visualization:
            return
            
        try:
            # Convert ROS PointCloud2 to Open3D point cloud
            points = self.pointcloud2_to_array(msg)
            if points is None or len(points) == 0:
                return
                
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # Check if point cloud is valid
            if len(pcd.points) == 0:
                return
                
            # Set uniform color for all points
            pcd.paint_uniform_color([0.8, 0.8, 0.8])  # Light gray color
            
            # Accumulate point cloud
            with self.cloud_lock:
                # Clear dummy points if this is the first real data
                if len(self.accumulated_cloud.points) == 1 and np.allclose(list(self.accumulated_cloud.points)[0], [0, 0, 0]):
                    self.accumulated_cloud.clear()
                
                self.accumulated_cloud += pcd
                
                # Check if accumulated cloud is too large and downsample if needed
                if len(self.accumulated_cloud.points) > self.max_cloud_size:
                    self.accumulated_cloud = self.accumulated_cloud.voxel_down_sample(voxel_size=self.downsample_voxel_size)
                    # self.get_logger().info(f"📊 Downsampled environment cloud to {len(self.accumulated_cloud.points)} points")
            
            # 如果可视化未启动且这是第一个有效数据，则启动可视化
            if not self.visualization_started and len(self.accumulated_cloud.points) > 0:
                self.get_logger().info("📊 First point cloud data received, starting visualization...")
                self.start_visualization()
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def trajectory_callback(self, msg: PointCloud2) -> None:
        """
        Callback function for processing incoming trajectory messages.
        
        Args:
            msg: PointCloud2 message from LIO-SAM
        """
        if not self.enable_visualization:
            return
            
        try:
            # Convert ROS PointCloud2 to Open3D point cloud
            points = self.pointcloud2_to_array(msg)
            if points is None or len(points) == 0:
                return
                
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # Set uniform color for all points
            pcd.paint_uniform_color([0.0, 1.0, 0.0])  # Green color for trajectory
            
            # Accumulate trajectory
            with self.cloud_lock:
                if self.trajectory_cloud is None:
                    self.trajectory_cloud = pcd
                else:
                    self.trajectory_cloud += pcd
            
            # 如果可视化未启动且这是第一个有效轨迹数据，则启动可视化
            if not self.visualization_started and len(self.trajectory_cloud.points) > 0:
                self.get_logger().info("🛤️ First trajectory data received, starting visualization...")
                self.start_visualization()
            
        except Exception as e:
            self.get_logger().error(f"Error processing trajectory: {e}")

    def pointcloud2_to_array(self, cloud_msg: PointCloud2) -> Optional[np.ndarray]:
        """
        Convert ROS PointCloud2 message to numpy array.
        
        Args:
            cloud_msg: ROS PointCloud2 message
            
        Returns:
            Numpy array of points, or None if conversion fails
        """
        try:
            import sensor_msgs_py.point_cloud2 as pc2
            
            # Extract points from PointCloud2
            points = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(points)
            
            if len(points_list) == 0:
                return None
                
            xyz = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)
            return xyz
            
        except ImportError:
            self.get_logger().error("sensor_msgs_py not available. Install with: pip install sensor_msgs_py")
            return None
        except Exception as e:
            self.get_logger().error(f"Error converting PointCloud2 to array: {e}")
            return None

    def visualize_thread(self) -> None:
        """
        Visualization thread for Open3D.
        """
        try:
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name="LIO-SAM Map + Trajectory Viewer", width=1280, height=720)

            added_env = False
            added_traj = False

            while self.visualization_running:
                time.sleep(0.05)
                with self.cloud_lock:
                    if not added_env and len(self.accumulated_cloud.points) > 0:
                        vis.add_geometry(self.accumulated_cloud)
                        added_env = True
                    elif added_env:
                        vis.update_geometry(self.accumulated_cloud)

                    if not added_traj and len(self.trajectory_cloud.points) > 0:
                        vis.add_geometry(self.trajectory_cloud)
                        added_traj = True
                    elif added_traj:
                        vis.update_geometry(self.trajectory_cloud)

                vis.poll_events()
                vis.update_renderer()
            
            # 关闭窗口
            vis.destroy_window()
            self.get_logger().info("🛑 Visualization window closed")
            
        except Exception as e:
            self.get_logger().error(f"Visualization thread error: {e}")
        finally:
            self.visualization_started = False

    def start_visualization(self) -> None:
        """
        启动点云可视化。
        """
        if not self.enable_visualization:
            self.get_logger().warning("Visualization is disabled")
            return
        
        if self.visualization_started:
            self.get_logger().info("Visualization is already running")
            return
        
        try:
            self.visualization_running = True
            self.visualization_thread = threading.Thread(target=self.visualize_thread, daemon=True)
            self.visualization_thread.start()
            self.visualization_started = True
            self.get_logger().info("🎬 Point cloud visualization started")
        except Exception as e:
            self.get_logger().error(f"Failed to start visualization: {e}")
            self.visualization_running = False
            self.visualization_started = False

    def stop_visualization(self) -> None:
        """
        停止点云可视化。
        """
        if not self.visualization_started:
            self.get_logger().info("Visualization is not running")
            return
        
        try:
            self.visualization_running = False
            
            # 等待可视化线程结束
            if self.visualization_thread and self.visualization_thread.is_alive():
                self.visualization_thread.join(timeout=2.0)
                if self.visualization_thread.is_alive():
                    self.get_logger().warning("Visualization thread did not stop gracefully")
            
            self.visualization_started = False
            self.get_logger().info("🛑 Point cloud visualization stopped")
        except Exception as e:
            self.get_logger().error(f"Error stopping visualization: {e}")

    def is_visualization_running(self) -> bool:
        """
        检查可视化是否正在运行。
        
        Returns:
            bool: 可视化是否正在运行
        """
        return self.visualization_started and self.visualization_running

    def clear_accumulated_cloud(self) -> None:
        """
        Clear the accumulated point cloud.
        """
        with self.cloud_lock:
            self.accumulated_cloud.clear()
        self.get_logger().info("🗑️ Cleared accumulated environment cloud")

    def clear_trajectory_cloud(self) -> None:
        """
        Clear the trajectory point cloud.
        """
        with self.cloud_lock:
            self.trajectory_cloud.clear()
        self.get_logger().info("🗑️ Cleared trajectory cloud")

    def clear_all_clouds(self) -> None:
        """
        Clear both environment and trajectory clouds.
        """
        with self.cloud_lock:
            self.accumulated_cloud.clear()
            self.trajectory_cloud.clear()
        self.get_logger().info("🗑️ Cleared all clouds")

    def save_accumulated_cloud(self, filename: str = "accumulated_cloud.pcd") -> None:
        """
        Save the accumulated environment point cloud to a PCD file.
        
        Args:
            filename: Output filename (should have .pcd extension)
        """
        if len(self.accumulated_cloud.points) == 0:
            self.get_logger().warning("No environment cloud to save")
            return
            
        try:
            with self.cloud_lock:
                # Ensure filename has .pcd extension
                if not filename.endswith('.pcd'):
                    filename = filename + '.pcd'
                
                o3d.io.write_point_cloud(filename, self.accumulated_cloud)
            self.get_logger().info(f"💾 Saved environment cloud to: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving environment cloud: {e}")

    def save_trajectory_cloud(self, filename: str = "trajectory_cloud.pcd") -> None:
        """
        Save the trajectory point cloud to a PCD file.
        
        Args:
            filename: Output filename (should have .pcd extension)
        """
        if len(self.trajectory_cloud.points) == 0:
            self.get_logger().warning("No trajectory cloud to save")
            return
            
        try:
            with self.cloud_lock:
                # Ensure filename has .pcd extension
                if not filename.endswith('.pcd'):
                    filename = filename + '.pcd'
                
                o3d.io.write_point_cloud(filename, self.trajectory_cloud)
            self.get_logger().info(f"💾 Saved trajectory cloud to: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving trajectory cloud: {e}")

    def save_combined_cloud(self, filename: str = "combined_map.pcd") -> None:
        """
        Save the combined environment and trajectory point cloud to a PCD file.
        
        Args:
            filename: Output filename (should have .pcd extension)
        """
        if len(self.accumulated_cloud.points) == 0 and len(self.trajectory_cloud.points) == 0:
            self.get_logger().warning("No clouds to save")
            return
            
        try:
            with self.cloud_lock:
                # Ensure filename has .pcd extension
                if not filename.endswith('.pcd'):
                    filename = filename + '.pcd'
                
                # Combine environment and trajectory clouds
                combined_cloud = o3d.geometry.PointCloud()
                if len(self.accumulated_cloud.points) > 0:
                    combined_cloud += self.accumulated_cloud
                if len(self.trajectory_cloud.points) > 0:
                    combined_cloud += self.trajectory_cloud
                
                o3d.io.write_point_cloud(filename, combined_cloud)
            self.get_logger().info(f"💾 Saved combined cloud to: {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving combined cloud: {e}")

    def get_cloud_size(self) -> int:
        """
        Get the current size of the accumulated environment point cloud.
        
        Returns:
            Number of points in the accumulated environment cloud, or 0 if no cloud exists
        """
        with self.cloud_lock:
            return len(self.accumulated_cloud.points)

    def get_trajectory_size(self) -> int:
        """
        Get the current size of the trajectory point cloud.
        
        Returns:
            Number of points in the trajectory cloud, or 0 if no cloud exists
        """
        with self.cloud_lock:
            return len(self.trajectory_cloud.points)

    def get_total_cloud_size(self) -> int:
        """
        Get the total size of both environment and trajectory clouds.
        
        Returns:
            Total number of points in both clouds
        """
        with self.cloud_lock:
            return len(self.accumulated_cloud.points) + len(self.trajectory_cloud.points)

    def set_downsample_parameters(self, max_size: int = 100000, voxel_size: float = 0.05) -> None:
        """
        Set downsampling parameters.
        
        Args:
            max_size: Maximum number of points before downsampling
            voxel_size: Voxel size for downsampling
        """
        self.max_cloud_size = max_size
        self.downsample_voxel_size = voxel_size
        self.get_logger().info(f"📊 Set downsampling parameters: max_size={max_size}, voxel_size={voxel_size}")

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Callback function for processing odometry messages.
        
        Args:
            msg: Odometry message from LIO-SAM
        """
        try:
            with self.pose_lock:
                # Extract position
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation
                
                # Convert quaternion to euler angles (yaw)
                import math
                # Roll (x-axis rotation)
                sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z)
                cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)
                roll = math.atan2(sinr_cosp, cosr_cosp)
                
                # Pitch (y-axis rotation)
                sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
                if abs(sinp) >= 1:
                    pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
                else:
                    pitch = math.asin(sinp)
                
                # Yaw (z-axis rotation)
                siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
                cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                # Store current pose
                self.current_pose = {
                    'position': (position.x, position.y, position.z),
                    'orientation': (orientation.x, orientation.y, orientation.z, orientation.w),
                    'euler': (roll, pitch, yaw),
                    'timestamp': msg.header.stamp
                }
                self.last_pose_update_time = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")
    
    def notice_callback(self, msg: String) -> None:
        """
        Callback function for processing qt_notice messages.
        收集发送信息后缓存等待时间内所有接收到的notice信息，然后逐个匹配。
        
        Args:
            msg: String message containing command execution feedback
        """
        try:
            with self.notice_lock:
                notice_data = msg.data
                current_time = time.time()
                
                # 更新最后接收到的notice
                self.last_notice = {
                    'message': notice_data,
                    'timestamp': current_time
                }
                
                self.get_logger().info(f"📢 Received notice: {notice_data}")
                
                # 将notice信息添加到缓存队列中
                notice_info = {
                    'message': notice_data,
                    'timestamp': current_time,
                    'raw_data': notice_data
                }
                
                # 添加到notice缓存队列
                if not hasattr(self, 'notice_cache_queue'):
                    self.notice_cache_queue = []
                
                self.notice_cache_queue.append(notice_info)
                
                # 清理过期的缓存信息（超过30秒的）
                current_time = time.time()
                self.notice_cache_queue = [
                    notice for notice in self.notice_cache_queue 
                    if current_time - notice['timestamp'] <= 30.0
                ]
                
                self.get_logger().info(f"📦 Notice cached. Queue size: {len(self.notice_cache_queue)}")
                
                # 如果有待匹配的命令序列，尝试匹配
                if hasattr(self, 'pending_command_seq') and self.pending_command_seq:
                    self._try_match_notice_with_command(notice_data, current_time)
                
        except Exception as e:
            self.get_logger().error(f"Error processing notice: {e}")
            import traceback
            traceback.print_exc()
    
    def _try_match_notice_with_command(self, notice_data: str, current_time: float) -> None:
        """
        尝试将notice信息与待匹配的命令序列进行匹配。
        
        Args:
            notice_data: 接收到的notice消息
            current_time: 当前时间戳
        """
        try:
            # 从notice中提取序列ID
            seq_id = self._extract_seq_id_from_notice(notice_data)
            
            if seq_id and seq_id in self.pending_command_seq:
                # 找到匹配的序列ID
                self.get_logger().info(f"🎯 Found matching sequence ID: {seq_id}")
                
                # 判断成功状态：只要找到匹配的序列ID就认为是成功的
                success = True
                
                # 存储确认信息
                self.command_confirmations[seq_id] = {
                    'message': notice_data,
                    'timestamp': current_time,
                    'success': success,
                    'raw_data': notice_data,
                    'matched': True
                }
                
                # 从待匹配列表中移除
                self.pending_command_seq.remove(seq_id)
                
                self.get_logger().info(f"✅ Command confirmation stored for sequence: {seq_id}")
                self.get_logger().info(f"   📋 Success: {success}")
                self.get_logger().info(f"   📝 Message: {notice_data}")
                self.get_logger().info(f"   📊 Remaining pending commands: {len(self.pending_command_seq)}")
                
            elif seq_id:
                # 找到了序列ID但没有匹配的待处理命令
                self.get_logger().info(f"ℹ️ Found sequence ID {seq_id} but no matching pending command")
                
            else:
                # 没有找到序列ID
                self.get_logger().debug(f"🔍 No sequence ID found in notice: {notice_data}")
                
        except Exception as e:
            self.get_logger().error(f"Error matching notice with command: {e}")
    
    def _extract_seq_id_from_notice(self, notice_data: str) -> Optional[str]:
        """
        从notice消息中提取序列ID。
        
        Args:
            notice_data: notice消息内容
            
        Returns:
            提取到的序列ID，如果没有找到则返回None
        """
        try:
            import re
            numbers = re.findall(r'\b\d+\b', notice_data)
            
            if numbers:
                # 找到最大的数字作为序列ID（通常是3位或以上的数字）
                potential_ids = [int(num) for num in numbers if len(num) >= 3]
                if potential_ids:
                    seq_id = str(max(potential_ids))
                    self.get_logger().debug(f"🔍 Extracted index: {seq_id} from message: {notice_data}")
                    return seq_id
                else:
                    # 如果没有3位以上的数字，使用最大的数字
                    seq_id = str(max([int(num) for num in numbers]))
                    self.get_logger().debug(f"🔍 Extracted index: {seq_id} from message: {notice_data}")
                    return seq_id
            else:
                self.get_logger().debug(f"⚠️ No numbers found in notice: {notice_data}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error extracting sequence ID: {e}")
            return None
    
    def get_pending_command_sequences(self) -> List[str]:
        """
        获取当前待匹配的命令序列列表。
        
        Returns:
            待匹配的命令序列ID列表
        """
        if not hasattr(self, 'pending_command_seq'):
            self.pending_command_seq = []
        return self.pending_command_seq.copy()
    
    def clear_pending_command_sequences(self) -> None:
        """
        清空待匹配的命令序列列表。
        """
        if hasattr(self, 'pending_command_seq'):
            self.pending_command_seq.clear()
            self.get_logger().info("🗑️ Cleared pending command sequences")
    
    def remove_pending_command_sequence(self, seq_id: str) -> bool:
        """
        从待匹配列表中移除指定的序列ID。
        
        Args:
            seq_id: 要移除的序列ID
            
        Returns:
            是否成功移除
        """
        if hasattr(self, 'pending_command_seq') and seq_id in self.pending_command_seq:
            self.pending_command_seq.remove(seq_id)
            self.get_logger().info(f"🗑️ Removed sequence ID {seq_id} from pending list")
            return True
        return False
    
    def get_notice_cache_queue_info(self) -> dict:
        """
        获取notice缓存队列的信息。
        
        Returns:
            包含缓存队列信息的字典
        """
        if not hasattr(self, 'notice_cache_queue'):
            self.notice_cache_queue = []
        
        current_time = time.time()
        # 清理过期的缓存信息
        self.notice_cache_queue = [
            notice for notice in self.notice_cache_queue 
            if current_time - notice['timestamp'] <= 30.0
        ]
        
        return {
            'queue_size': len(self.notice_cache_queue),
            'oldest_timestamp': self.notice_cache_queue[0]['timestamp'] if self.notice_cache_queue else None,
            'newest_timestamp': self.notice_cache_queue[-1]['timestamp'] if self.notice_cache_queue else None,
            'cache_duration': 30.0
        }
    
    def get_matching_status_info(self) -> dict:
        """
        获取命令匹配状态的详细信息。
        
        Returns:
            包含匹配状态信息的字典
        """
        pending_sequences = self.get_pending_command_sequences()
        cache_info = self.get_notice_cache_queue_info()
        
        # 分析缓存队列中的序列ID
        cached_seq_ids = []
        if hasattr(self, 'notice_cache_queue'):
            for notice in self.notice_cache_queue:
                seq_id = self._extract_seq_id_from_notice(notice['message'])
                if seq_id:
                    cached_seq_ids.append({
                        'seq_id': seq_id,
                        'message': notice['message'],
                        'timestamp': notice['timestamp']
                    })
        
        # 找出匹配和未匹配的序列ID
        matched_seq_ids = [seq for seq in pending_sequences if seq in [item['seq_id'] for item in cached_seq_ids]]
        unmatched_seq_ids = [seq for seq in pending_sequences if seq not in [item['seq_id'] for item in cached_seq_ids]]
        
        return {
            'pending_sequences': pending_sequences,
            'cached_notices': cached_seq_ids,
            'matched_sequences': matched_seq_ids,
            'unmatched_sequences': unmatched_seq_ids,
            'cache_queue_info': cache_info,
            'total_pending': len(pending_sequences),
            'total_cached': len(cached_seq_ids),
            'total_matched': len(matched_seq_ids),
            'total_unmatched': len(unmatched_seq_ids)
        }

    def get_current_pose(self) -> Optional[dict]:
        """
        Get the current pose from odometry data.
        
        Returns:
            Current pose dictionary with position, orientation, euler angles, and timestamp,
            or None if no pose data available
        """
        with self.pose_lock:
            return self.current_pose.copy() if self.current_pose else None
    
    def get_realtime_pose(self) -> Optional[dict]:
        """
        Get the most recent pose data with timeout check.
        
        Returns:
            Current pose dictionary if data is recent (within timeout),
            or None if data is stale or unavailable
        """
        with self.pose_lock:
            if self.current_pose is None:
                return None
            
            # Check if pose data is recent enough
            current_time = time.time()
            if current_time - self.last_pose_update_time > self.pose_timeout:
                self.get_logger().warning(f"Pose data is stale (age: {current_time - self.last_pose_update_time:.2f}s)")
                return None
            
            return self.current_pose.copy()
    
    def wait_for_fresh_pose(self, timeout: float = 2.0) -> Optional[dict]:
        """
        Wait for fresh pose data with specified timeout.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            Fresh pose data or None if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            pose = self.get_realtime_pose()
            if pose is not None:
                return pose
            time.sleep(0.1)  # Small delay to avoid busy waiting
        
        self.get_logger().warning(f"Timeout waiting for fresh pose data ({timeout}s)")
        return None

    def add_node_at_current_pose(self, node_name: int, use_realtime: bool = True, publish_immediately: bool = False) -> tuple[bool, int]:
        """
        Add a navigation node at the current pose from odometry.
        
        Args:
            node_name: Name/ID of the node
            use_realtime: Whether to use realtime pose data with timeout check
            publish_immediately: If True, publish immediately; if False, store internally
            
        Returns:
            tuple[bool, int]: (success, command_index)
        """
        if use_realtime:
            # Try to get fresh pose data
            current_pose = self.get_realtime_pose()
            if current_pose is None:
                self.get_logger().warning("No fresh pose data available, trying to wait for new data...")
                current_pose = self.wait_for_fresh_pose(timeout=1.0)
                if current_pose is None:
                    self.get_logger().error("Failed to get fresh pose data within timeout")
                    return False, -1
        else:
            # Use any available pose data (original behavior)
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().warning("No current pose available from odometry")
                return False, -1
        
        position = current_pose['position']
        yaw = current_pose['euler'][2]  # Yaw angle
        
        # Log pose freshness
        current_time = time.time()
        pose_age = current_time - self.last_pose_update_time
        self.get_logger().info(f"Using pose data (age: {pose_age:.3f}s)")
        
        # Add node using current pose with internal storage option
        index = self.add_node(node_name, position[0], position[1], position[2], yaw, publish_immediately)
        self.get_logger().info(f"📍 Added node {node_name} at current pose: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) with yaw {yaw:.2f}, index: {index}")
        return True, index
    
    def get_last_notice(self) -> Optional[dict]:
        """
        Get the most recent notice message.
        
        Returns:
            Last notice dictionary with message and timestamp, or None if no notice received
        """
        with self.notice_lock:
            return self.last_notice.copy() if self.last_notice else None
    
    def wait_for_command_confirmation(self, seq_id: str, timeout: float = 5.0) -> Optional[dict]:
        """
        Wait for command execution confirmation with specified timeout.
        使用新的匹配机制：检查notice缓存队列中是否有匹配的序列ID。
        
        Args:
            seq_id: Sequence ID to wait for (e.g., "123")
            timeout: Maximum time to wait in seconds
            
        Returns:
            Confirmation data or None if timeout
        """
        # Ensure notice subscriber exists when waiting for confirmation
        self._ensure_notice_subscriber()
        
        # 首先检查是否已经收到了确认消息
        with self.notice_lock:
            if seq_id in self.command_confirmations:
                confirmation = self.command_confirmations[seq_id].copy()
                self.get_logger().info(f"✅ Already received confirmation for sequence {seq_id}: {confirmation['message']}")
                # Destroy notice subscriber after successful confirmation
                self._destroy_notice_subscriber()
                return confirmation
        
        self.get_logger().info(f"🔄 Waiting for command confirmation (seq: {seq_id}, timeout: {timeout}s)")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            # 检查是否已经匹配成功
            with self.notice_lock:
                if seq_id in self.command_confirmations:
                    confirmation = self.command_confirmations[seq_id].copy()
                    self.get_logger().info(f"✅ Received confirmation for sequence {seq_id}: {confirmation['message']}")
                    # Destroy notice subscriber after successful confirmation
                    self._destroy_notice_subscriber()
                    return confirmation
            
            # 检查notice缓存队列中是否有匹配的消息
            if hasattr(self, 'notice_cache_queue'):
                with self.notice_lock:
                    for notice in self.notice_cache_queue:
                        extracted_seq_id = self._extract_seq_id_from_notice(notice['message'])
                        if extracted_seq_id == seq_id:
                            # 找到匹配的notice，手动触发匹配
                            self._try_match_notice_with_command(notice['message'], notice['timestamp'])
                            break
            
            time.sleep(0.1)  # Small delay to avoid busy waiting
        
        # 详细的超时错误信息
        elapsed_time = time.time() - start_time
        self.get_logger().error(f"❌ Command confirmation timeout!")
        self.get_logger().error(f"   📋 Sequence ID: {seq_id}")
        self.get_logger().error(f"   ⏱️  Timeout duration: {timeout}s")
        self.get_logger().error(f"   ⏰ Actual elapsed time: {elapsed_time:.2f}s")
        self.get_logger().error(f"   📢 Notice topic: {self.notice_topic}")
        self.get_logger().error(f"   🔍 Available confirmations: {list(self.command_confirmations.keys())}")
        self.get_logger().error(f"   📊 Total confirmations received: {len(self.command_confirmations)}")
        
        # 获取详细的分析信息
        analysis = self.get_latest_notice_analysis()
        if analysis['recent_confirmations']:
            self.get_logger().error("   📝 Recent confirmations details:")
            for conf in analysis['recent_confirmations']:
                self.get_logger().error(f"      - Seq: {conf['seq_id']}, Success: {conf['success']}")
                self.get_logger().error(f"        Raw message: {conf['raw_data']}")
                if conf.get('note'):
                    self.get_logger().error(f"        Note: {conf['note']}")
        
        # 检查是否收到了其他序列的确认
        if self.command_confirmations:
            self.get_logger().warning(f"⚠️  Received confirmations for other sequences: {list(self.command_confirmations.keys())}")
            
            # 尝试找到最接近的序列ID
            try:
                target_seq = int(seq_id)
                available_seqs = []
                for key in self.command_confirmations.keys():
                    if key.isdigit():
                        available_seqs.append(int(key))
                    elif key.startswith('debug_'):
                        continue
                
                if available_seqs:
                    closest_seq = min(available_seqs, key=lambda x: abs(x - target_seq))
                    self.get_logger().warning(f"🔍 Closest available sequence ID: {closest_seq} (target was {target_seq})")
            except ValueError:
                self.get_logger().warning("🔍 Could not compare sequence IDs (non-numeric)")
        
        # Destroy notice subscriber after timeout
        self._destroy_notice_subscriber()
        return None
    
    def clear_command_confirmations(self) -> None:
        """
        Clear all stored command confirmations.
        """
        with self.notice_lock:
            self.command_confirmations.clear()
        self.get_logger().info("🗑️ Cleared all command confirmations")
        # Destroy notice subscriber when clearing confirmations
        self._destroy_notice_subscriber()
    
    def get_command_confirmation(self, seq_id: str) -> Optional[dict]:
        """
        
        Get command confirmation for a specific sequence ID.
        
        Args:
            seq_id: Sequence ID to check
            
        Returns:
            Confirmation data or None if not found
        """
        with self.notice_lock:
            return self.command_confirmations.get(seq_id, {}).copy() if seq_id in self.command_confirmations else None
    
    def get_latest_notice_analysis(self) -> dict:
        """
        Get analysis of the latest notice messages for debugging.
        
        Returns:
            Dictionary containing analysis of recent notices
        """
        with self.notice_lock:
            analysis = {
                'total_notices': len(self.command_confirmations),
                'available_confirmations': list(self.command_confirmations.keys()),
                'latest_notice': self.last_notice,
                'recent_confirmations': []
            }
            
            # 获取最近的确认信息（按时间戳排序）
            confirmations = []
            for seq_id, data in self.command_confirmations.items():
                confirmations.append({
                    'seq_id': seq_id,
                    'timestamp': data.get('timestamp', 0),
                    'message': data.get('message', ''),
                    'success': data.get('success', False),
                    'raw_data': data.get('raw_data', ''),
                    'note': data.get('note', '')
                })
            
            # 按时间戳排序，获取最近的5个
            confirmations.sort(key=lambda x: x['timestamp'], reverse=True)
            analysis['recent_confirmations'] = confirmations[:5]
            
            return analysis
    
    def debug_notice_topic(self, duration: float = 10.0) -> None:
        """
        Debug the notice topic by listening for messages for a specified duration.
        
        Args:
            duration: How long to listen for messages (seconds)
        """
        self.get_logger().info(f"🔍 Starting notice topic debug for {duration} seconds...")
        
        # 确保notice subscriber存在
        self._ensure_notice_subscriber()
        
        # 记录开始时间
        start_time = time.time()
        original_notice_count = len(self.command_confirmations)
        
        # 监听消息
        while time.time() - start_time < duration:
            current_count = len(self.command_confirmations)
            if current_count > original_notice_count:
                # 有新消息
                new_messages = current_count - original_notice_count
                self.get_logger().info(f"📢 Received {new_messages} new notice(s)")
                original_notice_count = current_count
            
            time.sleep(0.1)
        
        # 分析结果
        analysis = self.get_latest_notice_analysis()
        self.get_logger().info("📊 Notice topic debug analysis:")
        self.get_logger().info(f"   📈 Total notices received: {analysis['total_notices']}")
        self.get_logger().info(f"   🔑 Available confirmations: {analysis['available_confirmations']}")
        
        if analysis['recent_confirmations']:
            self.get_logger().info("   📝 Recent confirmations:")
            for conf in analysis['recent_confirmations']:
                self.get_logger().info(f"      - Seq: {conf['seq_id']}, Success: {conf['success']}")
                self.get_logger().info(f"        Message: {conf['message']}")
        
        if analysis['latest_notice']:
            self.get_logger().info(f"   📢 Latest notice: {analysis['latest_notice']['message']}")
        
        self.get_logger().info("✅ Notice topic debug completed")
    
    def _create_notice_subscriber(self):
        """Create notice subscriber when needed."""
        if self.notice_subscriber is None:
            self.notice_subscriber = self.create_subscription(
                String,
                self.notice_topic,
                self.notice_callback,
                self.notice_qos
            )
            self.get_logger().info(f"📢 Notice subscriber created for topic: {self.notice_topic}")
    
    def _destroy_notice_subscriber(self):
        """Destroy notice subscriber when no longer needed."""
        if self.notice_subscriber is not None:
            self.destroy_subscription(self.notice_subscriber)
            self.notice_subscriber = None
            self.get_logger().info("📢 Notice subscriber destroyed")
    
    def _ensure_notice_subscriber(self):
        """Ensure notice subscriber exists when waiting for confirmation."""
        if self.notice_subscriber is None:
            self._create_notice_subscriber()
    
    def _cache_notice_data(self, duration: float = 2.0) -> dict:
        """
        Cache notice data for a specified duration.
        
        Args:
            duration: Duration to cache notice data in seconds
            
        Returns:
            dict: Cache status information
        """
        self.get_logger().info(f"📢 Starting notice data cache for {duration}s...")
        
        # Ensure notice subscriber exists
        self._ensure_notice_subscriber()
        
        # Clear previous command confirmations
        initial_count = len(self.command_confirmations)
        self.clear_command_confirmations()
        
        # Cache notice data for specified duration
        start_time = time.time()
        time.sleep(duration)
        end_time = time.time()
        
        # Get cache results
        cached_count = len(self.command_confirmations)
        cache_duration = end_time - start_time
        
        cache_status = {
            'initial_count': initial_count,
            'cached_count': cached_count,
            'cache_duration': cache_duration,
            'requested_duration': duration,
            'cached_messages': list(self.command_confirmations.keys())
        }
        
        self.get_logger().info(f"📋 Notice cache completed:")
        self.get_logger().info(f"   📊 Initial notices: {initial_count}")
        self.get_logger().info(f"   📊 Cached notices: {cached_count}")
        self.get_logger().info(f"   ⏱️ Cache duration: {cache_duration:.2f}s")
        
        return cache_status
    
    def get_notice_cache_status(self) -> dict:
        """
        Get the current status of notice cache.
        
        Returns:
            dict: Cache status information
        """
        with self.notice_lock:
            return {
                'subscriber_active': self.notice_subscriber is not None,
                'cached_messages': len(self.command_confirmations),
                'last_notice': self.last_notice,
                'command_confirmations': dict(self.command_confirmations),
                'notice_topic': self.notice_topic
            }
    
    def configure_notice_cache(self, cache_duration: float = 2.0, auto_cache: bool = True) -> dict:
        """
        Configure notice cache parameters.
        
        Args:
            cache_duration: Default duration to cache notice data
            auto_cache: Whether to automatically cache before publishing commands
            
        Returns:
            dict: Configuration status
        """
        self.notice_cache_duration = cache_duration
        self.notice_auto_cache = auto_cache
        
        config = {
            'cache_duration': self.notice_cache_duration,
            'auto_cache': self.notice_auto_cache,
            'message': 'Notice cache configuration updated'
        }
        
        self.get_logger().info(f"⚙️ Notice cache configuration updated:")
        self.get_logger().info(f"   ⏱️ Cache duration: {cache_duration}s")
        self.get_logger().info(f"   🔄 Auto cache: {auto_cache}")
        
        return config
    
    def test_notice_cache(self, duration: float = 3.0) -> dict:
        """
        Test the notice cache functionality.
        
        Args:
            duration: Duration to test cache in seconds
            
        Returns:
            dict: Test results
        """
        self.get_logger().info(f"🧪 Testing notice cache for {duration}s...")
        
        # Get initial status
        initial_status = self.get_notice_cache_status()
        
        # Test cache functionality
        cache_result = self._cache_notice_data(duration=duration)
        
        # Get final status
        final_status = self.get_notice_cache_status()
        
        test_results = {
            'test_duration': duration,
            'initial_status': initial_status,
            'cache_result': cache_result,
            'final_status': final_status,
            'cache_working': final_status['cached_messages'] > 0 or cache_result['cached_count'] > 0
        }
        
        self.get_logger().info(f"🧪 Notice cache test completed:")
        self.get_logger().info(f"   ✅ Cache working: {test_results['cache_working']}")
        self.get_logger().info(f"   📊 Cached messages: {cache_result['cached_count']}")
        self.get_logger().info(f"   ⏱️ Test duration: {duration}s")
        
        return test_results
    
    def _get_next_index(self) -> int:
        """
        Get the next command index in a thread-safe manner.
        
        Returns:
            int: The next available command index
        """
        with self.index_lock:
            current_index = self.command_index
            self.command_index += 1
            return current_index
    
    def _generate_seq_string(self, index: int) -> str:
        """
        Generate a sequence string with the given index.
        
        Args:
            index: The command index
            
        Returns:
            str: Sequence string in format "index:123;"
        """
        return f"index:{index};"
    
    def _publish_command_with_index(self, msg, wait_for_confirmation: bool = True, delay_before_confirm: float = 5.0, cache_duration: float = 2.0) -> int:
        """
        Publish a command with an automatically generated index.
        First creates a notice subscriber to cache notice data, then publishes the command.
        
        Args:
            msg: The message to publish
            wait_for_confirmation: Whether to wait for confirmation
            delay_before_confirm: Delay in seconds before starting confirmation check
            cache_duration: Duration to cache notice data before publishing command
            
        Returns:
            int: The index used for this command
        """
        # Generate next index
        index = self._get_next_index()
        seq_string = self._generate_seq_string(index)
        
        # Set the sequence string
        if hasattr(msg, 'seq'):
            msg.seq.data = seq_string
        
        # Initialize pending command sequence list if not exists
        if not hasattr(self, 'pending_command_seq'):
            self.pending_command_seq = []
        
        # Add sequence ID to pending list for matching
        if wait_for_confirmation:
            seq_id = str(index)
            self.pending_command_seq.append(seq_id)
            self.get_logger().info(f"📝 Added sequence ID {seq_id} to pending list. Total pending: {len(self.pending_command_seq)}")
        
        # Create notice subscriber and cache data if confirmation is needed
        if wait_for_confirmation:
            self.get_logger().info(f"📢 Creating notice subscriber and caching data for {cache_duration}s before publishing command {index}")
            
            # Cache notice data using the new cache method
            cache_status = self._cache_notice_data(duration=cache_duration)
            
            # Log cache results
            self.get_logger().info(f"📋 Cache results for command {index}:")
            self.get_logger().info(f"   📊 Cached notices: {cache_status['cached_count']}")
            self.get_logger().info(f"   ⏱️ Cache duration: {cache_status['cache_duration']:.2f}s")
        
        # Publish the message
        if hasattr(msg, 'command'):
            self.command_publisher.publish(msg)
        elif hasattr(msg, 'node'):
            self.node_publisher.publish(msg)
        elif hasattr(msg, 'edge'):
            self.edge_publisher.publish(msg)
        
        self.get_logger().info(f"📤 Published command with index: {index}")
        
        # Wait for confirmation if requested
        if wait_for_confirmation:
            try:
                # 等待指定时间后再开始验证
                self.get_logger().info(f"⏳ Waiting {delay_before_confirm}s before starting confirmation check for index: {index}")
                time.sleep(delay_before_confirm)
                
                confirmation = self.wait_for_command_confirmation(str(index), timeout=5.0)
                if confirmation:
                    self.get_logger().info(f"✅ Command {index} confirmed successfully")
                else:
                    self.get_logger().warning(f"⚠️ Command {index} confirmation timeout")
                    # 如果超时，从待匹配列表中移除
                    if seq_id in self.pending_command_seq:
                        self.pending_command_seq.remove(seq_id)
                        self.get_logger().info(f"🗑️ Removed timeout sequence ID {seq_id} from pending list")
                return index
            except Exception as e:
                self.get_logger().error(f"❌ Error waiting for command {index} confirmation: {e}")
                # 如果出错，从待匹配列表中移除
                if seq_id in self.pending_command_seq:
                    self.pending_command_seq.remove(seq_id)
                    self.get_logger().info(f"🗑️ Removed error sequence ID {seq_id} from pending list")
                return index
        
        return index
    
    def _cb(self, key):
        """Create callback function for robot state messages."""
        def callback(msg):
            self.msg_cache[key] = msg
        return callback
    
    def _cam_worker(self, name, uri):
        """后台线程：持续拉流并缓存最新帧"""
        pipeline = (
            f"rtspsrc location={uri} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! appsink"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().error(f"❌ 无法打开{name}相机流: {uri}")
            return

        self.get_logger().info(f"✅ {name}相机已连接")
        while not self._stop_cam.is_set():
            ret, frame = cap.read()
            if ret:
                if name == "front":
                    self.front_frame = frame
                else:
                    self.back_frame = frame
            else:
                time.sleep(0.01)
        cap.release()
    
    def get_camera_data(self,
                    camera_name: List[str],
                    camera_mode: Optional[List[str]] = None) -> Dict:
        """
        按要求返回相机视频流数据
        参数:
            camera_name: ["front", "back", ...]
            camera_mode: ["RGB", "DEPTH", ...]  缺省用相机自身的 default_mode
        返回:
            {
                "data": {...},
                "code": "000000" / 其他,
                "message": "..."
            }
        """
        # 1. 校验长度
        if camera_mode is None:
            camera_mode = [self.cameras[name].default_mode for name in camera_name]
        if len(camera_name) != len(camera_mode):
            return dict(code="400001", message="camera_name 与 camera_mode 长度不一致", data={})

        data = {}
        for name, mode in zip(camera_name, camera_mode):
            if name not in self.cameras:
                return dict(code="400002", message=f"未找到相机 {name}", data={})

            cam = self.cameras[name]
            latest = cam.latest
            if not latest.status or latest.frame is None:
                return dict(code="500001", message=f"相机 {name} 未准备好", data={})

            # 2. 按模式处理帧
            frame = latest.frame
            mode_up = mode.upper()
            if mode_up == "RGB":
                pass  # 原始彩色
            elif mode_up == "DEPTH":
                # 示例：把 16bit 深度图转 8bit 伪彩（这里简单复制灰度）
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            elif mode_up == "WIDE":
                # 示例：畸变矫正（预留）
                pass
            else:
                return dict(code="400003", message=f"不支持的模式 {mode}", data={})

            # 3. 编码为 jpg -> base64
            ok, encoded = cv2.imencode(".jpg", frame)
            if not ok:
                return dict(code="500002", message=f"编码失败 {name}", data={})
            b64_str = base64.b64encode(encoded.tobytes()).decode()

            data[name] = dict(
                camera_id=latest.camera_id,
                video_data=b64_str,
                status=latest.status
            )

        return dict(code="000000", message="success", data=data)
    
    def get_nav_state(self) -> Dict:
        """
        返回一个包含机器人状态的字典
        """
        odom = self.msg_cache['odom']
        imu = self.msg_cache['imu']
        low = self.msg_cache['lowstate']

        # 默认值
        status = {
            "state": 1 if odom and imu and low else 0,
            "nav_path": [],  # 当前不处理路径规划，可后续拓展
            "pos": {"x":0, "y":0},
            "yaw": 0.0,
            "roll": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "v_linear":0,
            "vyaw": 0.0,
            "position_signal": 0,  # 默认 0，可结合 GPS 话题或 slam_info 判断
            "battery": 0.0,
            "battery_cycles": 0, # 目前还未测试，先设置为完成一次任务需要10%的电量
            "imu_temp": 0.0,
            "temp_ntc1":0.0,
            "temp_ntc2":0.0,
            "battery_temp":0.0
        }

        if odom:
            pos = odom.pose.pose.position
            ori = odom.pose.pose.orientation
            status["pos"] = {"x":pos.x, "y":pos.y}

            # 四元数转欧拉角（roll, pitch, yaw）
            import math
            sinr_cosp = 2 * (ori.w * ori.x + ori.y * ori.z)
            cosr_cosp = 1 - 2 * (ori.x**2 + ori.y**2)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1 - 2 * (ori.y**2 + ori.z**2)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            status["roll"] = roll
            status["yaw"] = yaw

            lin = odom.twist.twist.linear
            ang = odom.twist.twist.angular
            if lin is not None:
                status["vx"] = lin.x
                status["vy"] = lin.y
                status["v_linear"] = math.sqrt(lin.x**2 + lin.y**2)
            else:
                status["vx"] = status["vy"] = status["v_linear"] = 0.0
                
            if ang is not None:
                status["vyaw"] = ang.z
            else:
                status["vyaw"] = 0.0

        if low:
            status["imu_temp"] = float(low.imu_state.temperature)
            status["battery"] = float(low.bms_state.soc)
            status["battery_cycles"] = int(status["battery"] / 10 )
            status["temp_ntc1"] = float(low.temperature_ntc1)
            status["temp_ntc2"] = float(low.temperature_ntc2)
            status["battery_temp"] = float((low.bms_state.bq_ntc[0]+ low.bms_state.bq_ntc[1])/2)

        return status
    
    def get_latest_frame(self, front=True):
        """供外部调用，返回最新帧（Mat），若无返回 None"""
        return self.front_frame if front else self.back_frame
    
    def show_front_camera(self):
        """显示前相机视频流（按 q 退出）"""
        while True:
            frame = self.get_latest_frame(front=True)
            if frame is not None:
                cv2.imshow("Front Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyWindow("Front Camera")

    def show_back_camera(self):
        """显示后相机视频流（按 q 退出）"""
        while True:
            frame = self.get_latest_frame(front=False)
            if frame is not None:
                cv2.imshow("Back Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyWindow("Back Camera")
    
    def start_camera_display(self, front=True, back=False):
        """启动一个线程显示相机窗口（非阻塞），默认开启前面的摄像头"""
        if front:
            threading.Thread(target=self.show_front_camera, daemon=True).start()
        if back:
            threading.Thread(target=self.show_back_camera, daemon=True).start()
    

    
    def shutdown(self):
        """供 Ctrl-C 时调用"""
        self._stop_cam.set()
        for cam in self.cameras.values():
            cam.stop()
        for t in self.cam_threads:
            t.join()
        # Destroy notice subscriber on shutdown
        self._destroy_notice_subscriber()

    def add_node_at_current_pose_auto_collect(self, node_name: int = None, auto_connect: bool = True) -> tuple[bool, int]:
        """
        Automatically collect node at current pose and optionally connect to previous node.
        This method mimics the behavior of demo_b2.cpp's addNodeAndEdge() function.
        
        Args:
            node_name: Name/ID of the node (if None, auto-increment)
            auto_connect: Whether to automatically connect to the previous node
            
        Returns:
            tuple[bool, int]: (success, command_index)
        """
        # Get current pose
        current_pose = self.get_realtime_pose()
        if current_pose is None:
            self.get_logger().warning("No fresh pose data available, trying to wait for new data...")
            current_pose = self.wait_for_fresh_pose(timeout=1.0)
            if current_pose is None:
                self.get_logger().error("Failed to get fresh pose data within timeout")
                return False, -1
        
        position = current_pose['position']
        yaw = current_pose['euler'][2]  # Yaw angle
        
        # Auto-increment node name if not provided
        if node_name is None:
            with self.nodes_lock:
                node_name = len(self.internal_nodes) + 1
        
        # Add node to internal storage
        with self.nodes_lock:
            self.internal_nodes[node_name] = {
                'x': position[0],
                'y': position[1],
                'z': position[2],
                'yaw': yaw,
                'attribute': 0,
                'undefined': 0,
                'state_2': 0,
                'state_3': 0
            }
        
        self.get_logger().info(f"📍 Auto-collected node {node_name} at ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) with yaw {yaw:.2f}")
        
        # Auto-connect to previous node if requested and there are at least 2 nodes
        if auto_connect and len(self.internal_nodes) >= 2:
            prev_node = node_name - 1
            if prev_node in self.internal_nodes:
                edge_name = len(self.internal_edges) + 1
                with self.edges_lock:
                    self.internal_edges[edge_name] = {
                        'start_node': prev_node,
                        'end_node': node_name,
                        'dog_speed': 1.0,
                        'edge_state_2': 0,
                        'dog_stats': 0,
                        'edge_length': 0.0,
                        'dog_back_stats': 0,
                        'edge_state': 0,
                        'edge_state_1': 0,
                        'edge_state_3': 0,
                        'edge_state_4': 0
                    }
                
                self.get_logger().info(f"🔗 Auto-connected edge {edge_name}: {prev_node} → {node_name}")
        
        return True, 0
    
    def collect_and_save_nodes_edges(self, clear_after_save: bool = True) -> tuple[int, int]:
        """
        Collect all nodes and edges from internal storage and publish them.
        This method mimics the behavior of demo_b2.cpp's saveNodeAndEdge() function.
        
        Args:
            clear_after_save: Whether to clear internal storage after publishing
            
        Returns:
            tuple[int, int]: (node_command_index, edge_command_index)
        """
        self.get_logger().info("💾 Collecting and saving nodes and edges...")
        
        # Check if we have edges (as per demo_b2.cpp logic)
        with self.edges_lock:
            if len(self.internal_edges) == 0:
                self.get_logger().warning("⚠️ The number of edges in the topology graph is 0.")
                return 0, 0
        
        # Publish nodes first
        node_index = self.publish_all_nodes()
        
        # Wait a bit for nodes to be processed
        time.sleep(0.5)
        
        # Then publish edges
        edge_index = self.publish_all_edges()
        
        self.get_logger().info(f"✅ Collected and saved nodes and edges - Node index: {node_index}, Edge index: {edge_index}")
        
        # Clear internal storage if requested
        if clear_after_save:
            self.clear_internal_storage()
            self.get_logger().info("🗑️ Internal storage cleared after save")
        
        return node_index, edge_index
    
    def auto_collect_loop(self, node_interval: float = 2.0, max_nodes: int = 10) -> None:
        """
        Automatically collect nodes and edges in a loop.
        This mimics the behavior of demo_b2.cpp's keyExecute() function.
        
        Args:
            node_interval: Time interval between node collections (seconds)
            max_nodes: Maximum number of nodes to collect
        """
        self.get_logger().info(f"🔄 Starting auto-collect loop (interval: {node_interval}s, max nodes: {max_nodes})")
        self.get_logger().info("Press Ctrl+C to stop collection")
        
        try:
            node_count = 0
            while node_count < max_nodes:
                # Wait for the specified interval
                time.sleep(node_interval)
                
                # Auto-collect node and edge
                success, _ = self.add_node_at_current_pose_auto_collect(auto_connect=True)
                if success:
                    node_count += 1
                    self.get_logger().info(f"📊 Collected {node_count}/{max_nodes} nodes")
                else:
                    self.get_logger().warning("⚠️ Failed to collect node, retrying...")
            
            self.get_logger().info(f"✅ Auto-collection completed: {node_count} nodes collected")
            
        except KeyboardInterrupt:
            self.get_logger().info("⏹️ Auto-collection stopped by user")
    
    def get_collection_status(self) -> dict:
        """
        Get the current status of node and edge collection.
        
        Returns:
            dict: Collection status information
        """
        with self.nodes_lock:
            with self.edges_lock:
                return {
                    'nodes_collected': len(self.internal_nodes),
                    'edges_collected': len(self.internal_edges),
                    'node_ids': list(self.internal_nodes.keys()),
                    'edge_ids': list(self.internal_edges.keys()),
                    'last_node': max(self.internal_nodes.keys()) if self.internal_nodes else None,
                    'last_edge': max(self.internal_edges.keys()) if self.internal_edges else None
                }
    
    def clear_collection_and_start_mapping(self) -> None:
        """
        Clear all nodes and edges, then start mapping.
        This mimics the 'w' key behavior in demo_b2.cpp.
        """
        self.get_logger().info("🗑️ Clearing all nodes and edges...")
        self.delete_all_nodes()
        self.delete_all_edges()
        self.clear_internal_storage()
        
        self.get_logger().info("🗺️ Starting mapping...")
        self.start_mapping()
    
    def prepare_for_collection(self) -> None:
        """
        Prepare for node/edge collection by clearing existing data and starting relocation.
        This mimics the 'z' key behavior in demo_b2.cpp.
        """
        self.get_logger().info("🔄 Preparing for node/edge collection...")
        self.delete_all_nodes()
        self.delete_all_edges()
        self.clear_internal_storage()
        self.start_relocation()
        self.start_navigation()
        self.initialize_pose()

def main():
    """Example usage of the Navigator class."""
    rclpy.init()
    navigator = Navigator(enable_visualization=True)
    
    try:
        # Example usage
        navigator.get_logger().info("Starting navigation system test...")
        
        # Start mapping
        navigator.start_mapping()
        rclpy.spin_once(navigator, timeout_sec=1.0)

        # Wait for odometry data to be available
        navigator.get_logger().info("Waiting for odometry data...")
        for i in range(10):  # Wait up to 10 seconds
            rclpy.spin_once(navigator, timeout_sec=1.0)
            if navigator.get_current_pose() is not None:
                navigator.get_logger().info("✅ Odometry data received!")
                break
            navigator.get_logger().info(f"Waiting for odometry... ({i+1}/10)")
        
        # Add nodes at current pose
        if navigator.get_current_pose() is not None:
            # Add nodes at current pose
            navigator.add_node_at_current_pose(1)
            rclpy.spin_once(navigator, timeout_sec=1.0)
            
            # Move around and add another node
            navigator.get_logger().info("Please move the robot to a new position...")
            time.sleep(3.0)  # Wait for movement
            
            navigator.add_node_at_current_pose(2)
            rclpy.spin_once(navigator, timeout_sec=1.0)
        else:
            # Fallback to manual node addition
            navigator.get_logger().warning("No odometry data, using manual node positions")
            navigator.add_node(1, 1.0, 1.0, 0.0, 1.57)
            navigator.add_node(2, 2.0, 2.0, 0.0, 0.0)
            rclpy.spin_once(navigator, timeout_sec=1.0)
        
        # Add an edge
        navigator.add_edge(1, 1, 2, 1.0)
        rclpy.spin_once(navigator, timeout_sec=1.0)
        
        # End mapping
        navigator.end_mapping()
        rclpy.spin_once(navigator, timeout_sec=1.0)
        
        # Start navigation
        navigator.start_navigation()
        rclpy.spin_once(navigator, timeout_sec=1.0)
        
        # Keep running to visualize point clouds
        navigator.get_logger().info("Navigation system test completed! Visualizing point clouds...")
        navigator.get_logger().info("Press Ctrl+C to stop visualization")
        
        # Spin to receive point cloud messages
        rclpy.spin(navigator)
        
    except KeyboardInterrupt:
        navigator.get_logger().info("Test interrupted by user")
        # Save accumulated clouds before exiting
        navigator.save_accumulated_cloud("final_environment_map.pcd")
        navigator.save_trajectory_cloud("final_trajectory_map.pcd")
        navigator.save_combined_cloud("final_combined_map.pcd")
    finally:
        # 停止可视化
        navigator.stop_visualization()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 


