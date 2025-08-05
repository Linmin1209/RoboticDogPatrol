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
                 odometry_topic: str = "/lio_sam_ros2/mapping/odometry",
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
        
        # Create publishers
        self.command_publisher = self.create_publisher(QtCommand, command_topic, 10)
        self.node_publisher = self.create_publisher(QtNode, add_node_topic, 10)
        self.edge_publisher = self.create_publisher(QtEdge, add_edge_topic, 10)
        self.query_result_node_publisher = self.create_publisher(String, "/query_result_node", 10)
        self.query_result_edge_publisher = self.create_publisher(String, "/query_result_edge", 10)
        
        # Current pose storage
        self.current_pose = None
        self.pose_lock = threading.Lock()
        self.last_pose_update_time = 0.0
        self.pose_timeout = 0.5  # 0.5 seconds timeout for pose data
        
        # Command execution feedback storage
        self.last_notice = None
        self.notice_lock = threading.Lock()
        self.command_confirmations = {}  # Store command confirmations by sequence
        
        # Create subscriber for odometry
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odometry_callback,
            1
        )
        
        # Create subscriber for qt_notice feedback
        self.notice_subscriber = self.create_subscription(
            String,
            notice_topic,
            self.notice_callback,
            10
        )
        
        # Setup QoS profile for robot state subscriptions
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
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
            # Create subscriber for point cloud
            self.cloud_subscriber = self.create_subscription(
                PointCloud2, 
                cloud_topic, 
                self.cloud_callback, 
                10
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
    
    def start_mapping(self, seq: str = "index:123;", attribute: int = 0) -> None:
        """
        Start the mapping process.
        
        Args:
            seq: Sequence identifier
            attribute: Attribute value (B2 fixed as 0)
        """
        msg = QtCommand()
        msg.command = 3  # Start mapping command
        msg.attribute = attribute
        msg.seq = String()
        msg.seq.data = seq
        
        self.command_publisher.publish(msg)
        self.get_logger().info("🗺️ Sent start mapping command")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def end_mapping(self, seq: str = "index:123;", 
                   floor_index: int = 0, pcdmap_index: int = 0) -> None:
        """
        End the mapping process.
        
        Args:
            seq: Sequence identifier
            floor_index: Floor index
            pcdmap_index: PCD map index
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
        msg.command = 4  # End mapping
        msg.floor_index.append(floor_index)
        msg.pcdmap_index.append(pcdmap_index)
        
        self.command_publisher.publish(msg)
        self.get_logger().info(f"✅ Sent end mapping command (floor={floor_index}, map={pcdmap_index})")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def start_navigation(self, seq: str = "index:123;") -> None:
        """
        Start navigation.
        
        Args:
            seq: Sequence identifier
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
        
        self.command_publisher.publish(msg)
        self.get_logger().info("🚀 Sent start navigation command")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def pause_navigation(self, seq: str = "index:123;") -> None:
        """
        Pause navigation.
        
        Args:
            seq: Sequence identifier
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
        msg.command = 13  # Pause navigation
        
        self.command_publisher.publish(msg)
        self.get_logger().info("⏸️ Sent pause navigation command")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def query_node(self, seq: str = "index:123;", attribute = 1) -> bool:
        """
        Query navigation nodes.
        
        Args:
            seq: Sequence identifier
            attribute: Query attribute (default: 1 for nodes)
            
        Returns:
            bool: True if command was sent and confirmed successfully, False otherwise
        """
        try:
            msg = QtCommand()
            msg.seq = String()
            msg.seq.data = seq
            msg.command = 2
            msg.attribute = attribute
            msg.floor_index.append(999)
            msg.node_edge_name.append(999)
            self.command_publisher.publish(msg)
            self.get_logger().info("▶️ Sent query node command")
            
            # Wait for command confirmation
            if seq and 'index:' in seq:
                try:
                    seq_id = seq.split('index:')[1].split(';')[0]
                    confirmation = self.wait_for_command_confirmation(seq_id, timeout=3.0)
                    if confirmation:
                        # Publish query result to feedback topic
                        result_msg = String()
                        result_msg.data = json.dumps({
                            "seq": seq,
                            "command": "query_node",
                            "attribute": attribute,
                            "status": "success" if confirmation.get('success', False) else "failed",
                            "message": confirmation.get('message', ''),
                            "timestamp": time.time()
                        })
                        self.query_result_node_publisher.publish(result_msg)
                        self.get_logger().info(f"📤 Published query node result: {confirmation.get('message', '')}")
                        return confirmation.get('success', False)
                    else:
                        # Publish timeout result
                        result_msg = String()
                        result_msg.data = json.dumps({
                            "seq": seq,
                            "command": "query_node",
                            "attribute": attribute,
                            "status": "timeout",
                            "message": "Command confirmation timeout",
                            "timestamp": time.time()
                        })
                        self.query_result_node_publisher.publish(result_msg)
                        self.get_logger().warning("⏰ Query node command confirmation timeout")
                        return False
                except Exception as e:
                    self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
                    return True  # Command was sent successfully even if confirmation failed
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending query node command: {e}")
            return False
    
    def query_edge(self, seq: str = "index:123;", attribute = 2) -> bool:
        """
        Query navigation edges.
        
        Args:
            seq: Sequence identifier
            attribute: Query attribute (default: 2 for edges)
            
        Returns:
            bool: True if command was sent and confirmed successfully, False otherwise
        """
        try:
            msg = QtCommand()
            msg.seq = String()
            msg.seq.data = seq
            msg.command = 2
            msg.attribute = attribute
            msg.floor_index.append(999)
            msg.node_edge_name.append(999)
            self.command_publisher.publish(msg)
            self.get_logger().info("▶️ Sent query edge command")
            
            # Wait for command confirmation
            if seq and 'index:' in seq:
                try:
                    seq_id = seq.split('index:')[1].split(';')[0]
                    confirmation = self.wait_for_command_confirmation(seq_id, timeout=3.0)
                    if confirmation:
                        # Publish query result to feedback topic
                        result_msg = String()
                        result_msg.data = json.dumps({
                            "seq": seq,
                            "command": "query_edge",
                            "attribute": attribute,
                            "status": "success" if confirmation.get('success', False) else "failed",
                            "message": confirmation.get('message', ''),
                            "timestamp": time.time()
                        })
                        self.query_result_edge_publisher.publish(result_msg)
                        self.get_logger().info(f"📤 Published query edge result: {confirmation.get('message', '')}")
                        return confirmation.get('success', False)
                    else:
                        # Publish timeout result
                        result_msg = String()
                        result_msg.data = json.dumps({
                            "seq": seq,
                            "command": "query_edge",
                            "attribute": attribute,
                            "status": "timeout",
                            "message": "Command confirmation timeout",
                            "timestamp": time.time()
                        })
                        self.query_result_edge_publisher.publish(result_msg)
                        self.get_logger().warning("⏰ Query edge command confirmation timeout")
                        return False
                except Exception as e:
                    self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
                    return True  # Command was sent successfully even if confirmation failed
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending query edge command: {e}")
            return False


    def recover_navigation(self, seq: str = "index:123;") -> None:
        """
        Recover/resume navigation.
        
        Args:
            seq: Sequence identifier
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
        msg.command = 14  # Recover navigation command
        
        self.command_publisher.publish(msg)
        self.get_logger().info("▶️ Sent recover navigation command")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def add_node(self, node_name: int, x: float, y: float, z: float = 0.0, 
                yaw: float = 1.57, seq: str = "index:123;") -> None:
        """
        Add a navigation node.
        
        Args:
            node_name: Name/ID of the node
            x: X coordinate
            y: Y coordinate
            z: Z coordinate (default: 0.0)
            yaw: Yaw angle in radians (default: 1.57)
            seq: Sequence identifier
        """
        msg = QtNode()
        msg.seq = String()
        msg.seq.data = seq
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
        
        self.node_publisher.publish(msg)
        self.get_logger().info(f"✅ Added node {node_name} at ({x}, {y}, {z}) with yaw {yaw}")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def delete_node(self, node_ids: List[int], seq: str = "index:123;") -> None:
        """
        Delete navigation nodes.
        
        Args:
            node_ids: List of node IDs to delete
            seq: Sequence identifier
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
        msg.command = 1  # Delete operation
        msg.attribute = 1  # Delete nodes
        msg.node_edge_name.extend(node_ids)
        
        self.command_publisher.publish(msg)
        self.get_logger().info(f"🗑️ Sent delete node command: {node_ids}")
    
    def add_edge(self, edge_name: int, start_node: int, end_node: int, 
                dog_speed: float = 1.0, seq: str = "index:123;") -> None:
        """
        Add a navigation edge.
        
        Args:
            edge_name: Name/ID of the edge
            start_node: Starting node ID
            end_node: Ending node ID
            dog_speed: Speed of the dog (default: 1.0)
            seq: Sequence identifier
        """
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
        
        self.edge_publisher.publish(msg)
        self.get_logger().info(f"✅ Added edge {edge_name} from node {start_node} to {end_node}")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def delete_edge(self, edge_ids: List[int], seq: str = "index:123;") -> None:
        """
        Delete navigation edges.
        
        Args:
            edge_ids: List of edge IDs to delete
            seq: Sequence identifier
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
        msg.command = 1  # Delete operation
        msg.attribute = 2  # Delete edges
        msg.node_edge_name.extend(edge_ids)
        
        self.command_publisher.publish(msg)
        self.get_logger().info(f"🗑️ Sent delete edge command: {edge_ids}")
    
    def pose_init(self, seq: str = "index:123;", 
                 translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> None:
        """
        Initialize pose.
        
        Args:
            seq: Sequence identifier
            translation: Translation vector (x, y, z)
            quaternion: Quaternion (x, y, z, w)
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
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
        
        self.command_publisher.publish(msg)
        self.get_logger().info(f"📍 Sent pose init command at {translation}")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def start_relocation(self, seq: str = "index:123;", attribute: int = 0) -> None:
        """
        Start relocation process.
        
        Args:
            seq: Sequence identifier
            attribute: Attribute value (B2 fixed as 0)
        """
        msg = QtCommand()
        msg.seq = String()
        msg.seq.data = seq
        msg.command = 6  # Start relocation command
        msg.attribute = attribute
        
        self.command_publisher.publish(msg)
        self.get_logger().info("📍 Sent start relocation command")
        
        # Wait for confirmation if sequence is provided
        if seq and 'index:' in seq:
            try:
                seq_id = seq.split('index:')[1].split(';')[0]
                self.wait_for_command_confirmation(seq_id, timeout=3.0)
            except Exception as e:
                self.get_logger().warning(f"Could not extract sequence ID for confirmation: {e}")
    
    def delete_all_nodes(self, seq: str = "index:123;") -> None:
        """
        Delete all nodes.
        
        Args:
            seq: Sequence identifier
        """
        self.delete_node([999], seq)
    
    def delete_all_edges(self, seq: str = "index:123;") -> None:
        """
        Delete all edges.
        
        Args:
            seq: Sequence identifier
        """
        self.delete_edge([999], seq)

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
        
        Args:
            msg: String message containing command execution feedback
        """
        try:
            with self.notice_lock:
                self.last_notice = {
                    'message': msg.data,
                    'timestamp': time.time()
                }
                
                # Parse the notice message to extract sequence and status
                # Expected format: "seq:index:123; status:success" or similar
                notice_data = msg.data
                self.get_logger().info(f"📢 Received notice: {notice_data}")
                
                # Extract sequence ID if present
                if 'index:' in notice_data:
                    try:
                        seq_start = notice_data.find('index:') + 6
                        seq_end = notice_data.find(';', seq_start)
                        if seq_end == -1:
                            seq_end = len(notice_data)
                        seq_id = notice_data[seq_start:seq_end].strip()
                        
                        # Store confirmation for this sequence
                        self.command_confirmations[seq_id] = {
                            'message': notice_data,
                            'timestamp': time.time(),
                            'success': 'success' in notice_data.lower() or 'ok' in notice_data.lower()
                        }
                        
                        self.get_logger().info(f"✅ Command confirmation stored for sequence: {seq_id}")
                    except Exception as e:
                        self.get_logger().warning(f"Could not parse sequence from notice: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing notice: {e}")

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

    def add_node_at_current_pose(self, node_name: int, seq: str = "index:123;", use_realtime: bool = True) -> bool:
        """
        Add a navigation node at the current pose from odometry.
        
        Args:
            node_name: Name/ID of the node
            seq: Sequence identifier
            use_realtime: Whether to use realtime pose data with timeout check
            
        Returns:
            True if node was added successfully, False if no pose data available
        """
        if use_realtime:
            # Try to get fresh pose data
            current_pose = self.get_realtime_pose()
            if current_pose is None:
                self.get_logger().warning("No fresh pose data available, trying to wait for new data...")
                current_pose = self.wait_for_fresh_pose(timeout=1.0)
                if current_pose is None:
                    self.get_logger().error("Failed to get fresh pose data within timeout")
                    return False
        else:
            # Use any available pose data (original behavior)
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().warning("No current pose available from odometry")
                return False
        
        position = current_pose['position']
        yaw = current_pose['euler'][2]  # Yaw angle
        
        # Log pose freshness
        current_time = time.time()
        pose_age = current_time - self.last_pose_update_time
        self.get_logger().info(f"Using pose data (age: {pose_age:.3f}s)")
        
        # Add node using current pose
        self.add_node(node_name, position[0], position[1], position[2], yaw, seq)
        self.get_logger().info(f"📍 Added node {node_name} at current pose: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) with yaw {yaw:.2f}")
        return True
    
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
        
        Args:
            seq_id: Sequence ID to wait for (e.g., "123")
            timeout: Maximum time to wait in seconds
            
        Returns:
            Confirmation data or None if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.notice_lock:
                if seq_id in self.command_confirmations:
                    confirmation = self.command_confirmations[seq_id].copy()
                    self.get_logger().info(f"✅ Received confirmation for sequence {seq_id}: {confirmation['message']}")
                    return confirmation
            time.sleep(0.1)  # Small delay to avoid busy waiting
        
        self.get_logger().warning(f"⏰ Timeout waiting for command confirmation (seq: {seq_id}, timeout: {timeout}s)")
        return None
    
    def clear_command_confirmations(self) -> None:
        """
        Clear all stored command confirmations.
        """
        with self.notice_lock:
            self.command_confirmations.clear()
        self.get_logger().info("🗑️ Cleared all command confirmations")
    
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

