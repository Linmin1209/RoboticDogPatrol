#!/usr/bin/env python3
"""
Interactive Navigator Example with Keyboard Controls

This script provides an interactive interface for controlling the Navigator class
using keyboard inputs for various navigation operations.
"""

import os
import sys

# 设置ROS2 DDS环境变量来解决消息大小限制问题
os.environ['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '0'  # 禁用XML配置，使用代码中的QoS设置
os.environ['RMW_FASTRTPS_MAX_HISTORY_DEPTH'] = '10000'
os.environ['RMW_FASTRTPS_MAX_SYNCHRONOUS_DISCOVERY_RETRIES'] = '10'

import rclpy
import time
from navigator import Navigator
import threading

class InteractiveNavigator:
    """
    Interactive navigator with keyboard controls.
    """
    
    def __init__(self):
        """Initialize the interactive navigator."""
        rclpy.init()
        self.navigator = Navigator(enable_visualization=False)  # 禁用可视化以减少消息大小
        self.running = True
        self.node_counter = 1
        self.edge_counter = 1
        # 移除可视化启动，放到 run() 里
        # self.navigator.start_visualization()
        
        print("🎮 Interactive Navigator Control")
        print("=" * 50)
        print("Available commands:")
        print("  [M] Start/End Mapping")
        print("  [N] Add Node at Current Pose")
        print("  [E] Add Edge between Nodes")
        print("  [S] Start Navigation")
        print("  [P] Pause Navigation")
        print("  [R] Recover Navigation")
        print("  [I] Initialize Pose")
        print("  [CN] Close All Nodes")
        print("  [L] Start Relocation and Collecting Node and Edge")
        print("  [C] Clear Environment Cloud")
        print("  [T] Clear Trajectory Cloud")
        print("  [A] Clear All Clouds")
        print("  [V] Save Environment Cloud")
        print("  [R] Save Trajectory Cloud")
        print("  [S] Save Combined Cloud")
        print("  [G] Get Current Pose")
        print("  [I] Get Cloud Info")
        print("  [D] Set Downsample Parameters")
        print("  [RT] Check Realtime Pose Status")
        print("  [FP] Fixed Point Navigation")
        print("  [CA] Camera Control")
        print("  [H] Show Help")
        print("  [Q] Quit")
        print("=" * 50)
    
    def get_user_input(self, prompt: str) -> str:
        """Get user input with prompt."""
        return input(prompt)
    
    def get_int_input(self, prompt: str, default: int = 1) -> int:
        """Get integer input with default value."""
        try:
            user_input = input(f"{prompt} (default: {default}): ").strip()
            return int(user_input) if user_input else default
        except ValueError:
            print(f"Invalid input, using default: {default}")
            return default
    
    def get_float_input(self, prompt: str, default: float = 0.0) -> float:
        """Get float input with default value."""
        try:
            user_input = input(f"{prompt} (default: {default}): ").strip()
            return float(user_input) if user_input else default
        except ValueError:
            print(f"Invalid input, using default: {default}")
            return default
    
    def start_mapping(self):
        """Start mapping operation."""
        print("🗺️ Starting mapping...")
        self.navigator.start_mapping()
        print("✅ Mapping started")

    def close_all_nodes(self):
        """Close all nodes."""
        print("🗑️ Closing all nodes...")
        self.navigator.close_all_nodes()
        print("✅ All nodes closed")
    
    def end_mapping(self):
        """End mapping operation."""
        floor_index = self.get_int_input("Enter floor index", 0)
        pcdmap_index = self.get_int_input("Enter PCD map index", 0)
        
        print("🏁 Ending mapping...")
        self.navigator.end_mapping(floor_index=floor_index, pcdmap_index=pcdmap_index)
        print("✅ Mapping ended")
    
    def add_node_at_current_pose(self):
        """Add node at current pose."""
        node_name = self.get_int_input("Enter node name/ID", self.node_counter)
        
        # Ask user if they want to use realtime pose data
        use_realtime = input("Use realtime pose data (y/n, default: y): ").strip().lower()
        use_realtime = use_realtime != 'n'  # Default to True unless explicitly 'n'
        
        print("📍 Adding node at current pose...")
        print("Options:")
        print("  1. Store internally (default)")
        print("  2. Publish immediately")
        
        choice = input("Enter choice (1-2, default: 1): ").strip()
        publish_immediately = (choice == "2")
        
        if use_realtime:
            print("🔄 Using realtime pose data with timeout check...")
        else:
            print("📊 Using any available pose data...")
            
        success, index = self.navigator.add_node_at_current_pose(node_name, use_realtime=use_realtime, publish_immediately=publish_immediately)
        if success:
            if publish_immediately:
                print(f"✅ Node {node_name} published immediately with index: {index}")
            else:
                print(f"📍 Node {node_name} stored internally")
            self.node_counter += 1
        else:
            print("❌ Failed to add node - no pose data available")
    
    def add_edge(self):
        """Add edge between nodes."""
        start_node = self.get_int_input("Enter start node ID", 1)
        end_node = self.get_int_input("Enter end node ID", 2)
        edge_name = self.get_int_input("Enter edge name/ID", self.edge_counter)
        speed = self.get_float_input("Enter dog speed", 1.0)
        
        print("🔗 Adding edge...")
        print("Options:")
        print("  1. Store internally (default)")
        print("  2. Publish immediately")
        
        choice = input("Enter choice (1-2, default: 1): ").strip()
        publish_immediately = (choice == "2")
        
        print(f"🔗 Adding edge {edge_name} from node {start_node} to {end_node}...")
        index = self.navigator.add_edge(edge_name, start_node, end_node, speed, publish_immediately=publish_immediately)
        
        if publish_immediately:
            print(f"✅ Edge {edge_name} published immediately with index: {index}")
        else:
            print(f"🔗 Edge {edge_name} stored internally")
        self.edge_counter += 1
    
    def add_edge_with_restriction(self):
        """Add edge with restriction check."""
        start_node = self.get_int_input("Enter start node ID", 1)
        end_node = self.get_int_input("Enter end node ID", 2)
        edge_name = self.get_int_input("Enter edge name/ID", self.edge_counter)
        speed = self.get_float_input("Enter dog speed", 1.0)
        
        # 获取起点和终点位置（这里简化处理，实际应用中需要从节点数据中获取）
        start_x = self.get_float_input("Enter start X position", 0.0)
        start_y = self.get_float_input("Enter start Y position", 0.0)
        end_x = self.get_float_input("Enter end X position", 1.0)
        end_y = self.get_float_input("Enter end Y position", 1.0)
        
        if self.navigator.add_edge_with_restriction_check(edge_name, start_node, end_node, speed, 
                                                        start_pos=(start_x, start_y), end_pos=(end_x, end_y)):
            print(f"✅ Edge {edge_name} added successfully")
            self.edge_counter += 1
        else:
            print("❌ Failed to add edge - path intersects restricted area")
    
    def start_navigation(self):
        """Start navigation."""
        print("🚀 Starting navigation...")
        self.navigator.start_navigation()
        print("✅ Navigation started")

    def start_navigation_loop(self):
        """Start navigation loop."""
        print("🚀 Starting navigation loop...")
        self.navigator.default_navigation_loop()
        print("✅ Navigation loop started")
    
    def pause_navigation(self):
        """Pause navigation."""
        print("⏸️ Pausing navigation...")
        self.navigator.pause_navigation()
        print("✅ Navigation paused")
    
    def recover_navigation(self):
        """Recover navigation."""
        print("▶️ Recovering navigation...")
        self.navigator.recover_navigation()
        print("✅ Navigation recovered")
    
    def initialize_pose(self):
        """Initialize pose."""
        print("📍 Initializing pose...")
        
        self.navigator.pose_init()
        print("✅ Pose initialized")
    
    def start_relocation(self):
        """Start relocation."""
        print("📍 Starting relocation...")
        self.navigator.start_relocation()
        print("✅ Relocation started")
    
    def clear_cloud(self):
        """Clear accumulated environment point cloud."""
        print("🗑️ Clearing accumulated environment cloud...")
        self.navigator.clear_accumulated_cloud()
        print("✅ Environment cloud cleared")
    
    def clear_trajectory(self):
        """Clear trajectory point cloud."""
        print("🗑️ Clearing trajectory cloud...")
        self.navigator.clear_trajectory_cloud()
        print("✅ Trajectory cloud cleared")
    
    def clear_all_clouds(self):
        """Clear all point clouds."""
        print("🗑️ Clearing all clouds...")
        self.navigator.clear_all_clouds()
        print("✅ All clouds cleared")
    
    def save_cloud(self):
        """Save accumulated environment point cloud."""
        filename = self.get_user_input("Enter filename (without .pcd extension): ")
        if not filename:
            filename = "environment_map"
        
        print(f"💾 Saving environment cloud to {filename}.pcd...")
        self.navigator.save_accumulated_cloud(filename)
        print("✅ Environment cloud saved")
    
    def save_trajectory(self):
        """Save trajectory point cloud."""
        filename = self.get_user_input("Enter filename (without .pcd extension): ")
        if not filename:
            filename = "trajectory_map"
        
        print(f"💾 Saving trajectory cloud to {filename}.pcd...")
        self.navigator.save_trajectory_cloud(filename)
        print("✅ Trajectory cloud saved")
    
    def save_combined(self):
        """Save combined environment and trajectory clouds."""
        filename = self.get_user_input("Enter filename (without .pcd extension): ")
        if not filename:
            filename = "combined_map"
        
        print(f"💾 Saving combined cloud to {filename}.pcd...")
        self.navigator.save_combined_cloud(filename)
        print("✅ Combined cloud saved")
    
    def get_current_pose(self):
        """Get and display current pose."""
        pose = self.navigator.get_current_pose()
        if pose:
            pos = pose['position']
            euler = pose['euler']
            print(f"📍 Current Pose:")
            print(f"   Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            print(f"   Orientation (Euler): Roll={euler[0]:.3f}, Pitch={euler[1]:.3f}, Yaw={euler[2]:.3f}")
            print(f"   Timestamp: {pose['timestamp']}")
        else:
            print("❌ No pose data available")
    
    def get_cloud_info(self):
        """Get and display cloud information."""
        env_size = self.navigator.get_cloud_size()
        traj_size = self.navigator.get_trajectory_size()
        total_size = self.navigator.get_total_cloud_size()
        
        print(f"📊 Point Cloud Information:")
        print(f"   Environment cloud: {env_size:,} points")
        print(f"   Trajectory cloud: {traj_size:,} points")
        print(f"   Total points: {total_size:,}")
        print(f"   Max size before downsampling: {self.navigator.max_cloud_size:,}")
        print(f"   Downsample voxel size: {self.navigator.downsample_voxel_size}")
    
    def set_downsample_params(self):
        """Set downsampling parameters."""
        max_size = self.get_int_input("Enter max cloud size", self.navigator.max_cloud_size)
        voxel_size = self.get_float_input("Enter voxel size", self.navigator.downsample_voxel_size)
        
        self.navigator.set_downsample_parameters(max_size, voxel_size)
        print("✅ Downsampling parameters updated")
    
    def start_visualization(self):
        """Start point cloud visualization."""
        print("🎬 Starting point cloud visualization...")
        self.navigator.start_visualization()
        print("✅ Visualization started")
    
    def stop_visualization(self):
        """Stop point cloud visualization."""
        print("🛑 Stopping point cloud visualization...")
        self.navigator.stop_visualization()
        print("✅ Visualization stopped")
    
    def check_visualization_status(self):
        """Check visualization status."""
        status = self.navigator.is_visualization_running()
        print(f"📊 Visualization status: {'🟢 Running' if status else '🔴 Stopped'}")
        print(f"   Enabled: {self.navigator.enable_visualization}")
        print(f"   Started: {self.navigator.visualization_started}")
        print(f"   Running: {self.navigator.visualization_running}")
    
    def check_realtime_pose_status(self):
        """Check realtime pose status."""
        print("🔄 Checking realtime pose status...")
        
        # Get current pose
        current_pose = self.navigator.get_current_pose()
        if current_pose is None:
            print("❌ No pose data available")
            return
        
        # Get realtime pose
        realtime_pose = self.navigator.get_realtime_pose()
        if realtime_pose is None:
            print("⚠️ Pose data is stale (older than timeout)")
        else:
            print("✅ Fresh pose data available")
        
        # Display pose information
        position = current_pose['position']
        euler = current_pose['euler']
        print(f"📍 Current Position: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")
        print(f"🔄 Current Orientation (roll, pitch, yaw): ({euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f})")
        
        # Show pose age
        import time
        current_time = time.time()
        pose_age = current_time - self.navigator.last_pose_update_time
        print(f"⏰ Pose data age: {pose_age:.3f} seconds")
        
        if pose_age > self.navigator.pose_timeout:
            print("⚠️ Warning: Pose data is older than timeout threshold")
        else:
            print("✅ Pose data is within acceptable age range")
    
    def fixed_point_navigation(self):
        """Execute fixed point navigation."""
        print("🎯 Fixed Point Navigation")
        print("=" * 30)
        
        # Get target coordinates
        x = self.get_float_input("Enter target X coordinate", 5.0)
        y = self.get_float_input("Enter target Y coordinate", 3.0)
        yaw = self.get_float_input("Enter target Yaw angle (radians)", 0.0)
        goal_node_id = self.get_int_input("Enter target node ID", None)
        map_name = input("Enter map name (default: default): ").strip() or "default"
        
        print(f"🎯 Navigating to point: ({x:.2f}, {y:.2f}), angle: {yaw:.2f}°, map: {map_name}")
        
        # Execute navigation
        if self.navigator.navigate_to_point(x, y, yaw, goal_node_id, map_name):
            print("✅ Fixed point navigation started successfully")
            
            # Show current pose for reference
            current_pose = self.navigator.get_current_pose()
            if current_pose:
                pos = current_pose['position']
                print(f"📍 Starting from: ({pos[0]:.2f}, {pos[1]:.2f})")
                
                # Calculate distance
                distance = ((x - pos[0]) ** 2 + (y - pos[1]) ** 2) ** 0.5
                print(f"📏 Distance to target: {distance:.2f} meters")
        else:
            print("❌ Failed to start fixed point navigation")
    
    def camera_control(self):
        """Camera control menu."""
        print("📷 Camera Control")
        print("=" * 20)
        print("1. Show front camera")
        print("2. Show back camera")
        print("3. Start camera display (front)")
        print("4. Start camera display (back)")
        print("5. Get camera data")
        print("6. Back to main menu")
        
        choice = input("Enter choice (1-6): ").strip()
        
        if choice == '1':
            print("📷 Starting front camera display...")
            self.navigator.show_front_camera()
        elif choice == '2':
            print("📷 Starting back camera display...")
            self.navigator.show_back_camera()
        elif choice == '3':
            print("📷 Starting front camera display in background...")
            self.navigator.start_camera_display(front=True, back=False)
        elif choice == '4':
            print("📷 Starting back camera display in background...")
            self.navigator.start_camera_display(front=False, back=True)
        elif choice == '5':
            self.get_camera_data()
        elif choice == '6':
            return
        else:
            print("❌ Invalid choice")
    
    def get_camera_data(self):
        """Get camera data."""
        print("📷 Getting camera data...")
        
        # Get available cameras
        camera_names = list(self.navigator.cameras.keys())
        if not camera_names:
            print("❌ No cameras available")
            return
        
        print(f"📷 Available cameras: {camera_names}")
        
        # Get data for all cameras
        try:
            camera_data = self.navigator.get_camera_data(camera_names)
            
            print("📊 Camera Data:")
            for camera_name, data in camera_data.items():
                print(f"  📷 {camera_name}:")
                print(f"    Status: {'✅ Active' if data['status'] else '❌ Inactive'}")
                print(f"    Mode: {data['mode']}")
                print(f"    Frame size: {data['frame_size']}")
                print(f"    Timestamp: {data['timestamp']}")
                
        except Exception as e:
            print(f"❌ Error getting camera data: {e}")
    
    def delete_node(self):
        """Delete nodes."""
        print("🗑️ Delete Nodes")
        print("=" * 15)
        
        node_ids_input = input("Enter node IDs to delete (comma-separated): ").strip()
        if not node_ids_input:
            print("❌ No node IDs provided")
            return
        
        try:
            node_ids = [int(x.strip()) for x in node_ids_input.split(',')]
            print(f"🗑️ Deleting nodes: {node_ids}")
            self.navigator.delete_node(node_ids)
            print("✅ Nodes deleted successfully")
        except ValueError:
            print("❌ Invalid node ID format")
        except Exception as e:
            print(f"❌ Error deleting nodes: {e}")
    
    def delete_edge(self):
        """Delete edges."""
        print("🗑️ Delete Edges")
        print("=" * 15)
        
        edge_ids_input = input("Enter edge IDs to delete (comma-separated): ").strip()
        if not edge_ids_input:
            print("❌ No edge IDs provided")
            return
        
        try:
            edge_ids = [int(x.strip()) for x in edge_ids_input.split(',')]
            print(f"🗑️ Deleting edges: {edge_ids}")
            self.navigator.delete_edge(edge_ids)
            print("✅ Edges deleted successfully")
        except ValueError:
            print("❌ Invalid edge ID format")
        except Exception as e:
            print(f"❌ Error deleting edges: {e}")
    
    def delete_all_nodes(self):
        """Delete all nodes."""
        print("🗑️ Delete All Nodes")
        print("=" * 20)
        
        confirm = input("Are you sure you want to delete ALL nodes? (y/n): ").strip().lower()
        if confirm == 'y':
            print("🗑️ Deleting all nodes...")
            self.navigator.delete_all_nodes()
            print("✅ All nodes deleted successfully")
        else:
            print("❌ Operation cancelled")
    
    def delete_all_edges(self):
        """Delete all edges."""
        print("🗑️ Delete All Edges")
        print("=" * 20)
        
        confirm = input("Are you sure you want to delete ALL edges? (y/n): ").strip().lower()
        if confirm == 'y':
            print("🗑️ Deleting all edges...")
            self.navigator.delete_all_edges()
            print("✅ All edges deleted successfully")
        else:
            print("❌ Operation cancelled")
    
    def query_nodes(self):
        """Query nodes."""
        print("🔍 Query Nodes")
        print("=" * 15)
        if self.navigator.query_node():
            print("✅ Node query successful")
        else:
            print("❌ Node query failed")
    
    def query_edges(self):
        """Query edges."""
        print("🔍 Query Edges")
        print("=" * 15)
        
        if self.navigator.query_edge():
            print("✅ Edge query successful")
        else:
            print("❌ Edge query failed")
    
    def query_network_overview(self):
        """Query complete network overview (nodes and edges)."""
        print("🌐 Network Overview")
        print("=" * 25)
        try:
            # Query both nodes and edges
            nodes = self.navigator.query_node()
            edges = self.navigator.query_edge()
            
            print("📊 Network Summary:")
            print(f"  📍 Total nodes: {len(nodes) if nodes else 0}")
            print(f"  🔗 Total edges: {len(edges) if edges else 0}")
            
            if nodes and edges:
                # Calculate network connectivity
                connected_nodes = set()
                for edge_data in edges.values():
                    connected_nodes.add(edge_data.get('start_node'))
                    connected_nodes.add(edge_data.get('end_node'))
                
                isolated_nodes = set(nodes.keys()) - connected_nodes
                
                print(f"  🔗 Connected nodes: {len(connected_nodes)}")
                print(f"  📍 Isolated nodes: {len(isolated_nodes)}")
                
                if isolated_nodes:
                    print(f"  📍 Isolated node IDs: {sorted(isolated_nodes)}")
                
                # Show node details
                print("\n📍 Node Details:")
                for node_id, node_data in nodes.items():
                    pos = node_data.get('position', {})
                    print(f"  Node {node_id}: ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f}, {pos.get('z', 0):.2f})")
                
                # Show edge details
                print("\n🔗 Edge Details:")
                for edge_id, edge_data in edges.items():
                    print(f"  Edge {edge_id}: {edge_data.get('start_node')} → {edge_data.get('end_node')} (speed: {edge_data.get('speed', 0):.2f} m/s)")
                
                # Find disconnected components
                print("\n🔍 Connectivity Analysis:")
                if len(connected_nodes) > 0:
                    print(f"  ✅ Network is connected with {len(connected_nodes)} nodes")
                else:
                    print("  ⚠️  Network has no connections")
                
                if len(isolated_nodes) > 0:
                    print(f"  ⚠️  {len(isolated_nodes)} isolated nodes found")
                
            elif nodes and not edges:
                print("  ⚠️  Nodes exist but no edges defined")
            elif edges and not nodes:
                print("  ⚠️  Edges exist but no nodes defined")
            else:
                print("  📭 No nodes or edges found")
                
        except Exception as e:
            print(f"❌ Error querying network overview: {e}")
    
    def query_path_analysis(self):
        """Analyze possible paths between nodes."""
        print("🛤️ Path Analysis")
        print("=" * 20)
        
        try:
            nodes = self.navigator.query_node()
            edges = self.navigator.query_edge()
            
            if not nodes or not edges:
                print("❌ Need both nodes and edges for path analysis")
                return
            
            print("Options:")
            print("  1. Find all possible paths")
            print("  2. Find shortest path between two nodes")
            print("  3. Find all paths from a specific node")
            
            choice = input("Enter choice (1-3, default: 1): ").strip()
            
            if choice == "1" or choice == "":
                self.find_all_paths(nodes, edges)
            elif choice == "2":
                self.find_shortest_path(nodes, edges)
            elif choice == "3":
                self.find_paths_from_node(nodes, edges)
            else:
                print("❌ Invalid choice")
                
        except Exception as e:
            print(f"❌ Error in path analysis: {e}")
    
    def find_all_paths(self, nodes, edges):
        """Find all possible paths in the network."""
        print("🔍 Finding all possible paths...")
        
        # Build adjacency list
        adjacency = {}
        for edge_id, edge_data in edges.items():
            start = edge_data.get('start_node')
            end = edge_data.get('end_node')
            if start not in adjacency:
                adjacency[start] = []
            if end not in adjacency:
                adjacency[end] = []
            adjacency[start].append(end)
            adjacency[end].append(start)  # Assuming bidirectional
        
        # Find all paths (simplified - just direct connections)
        paths = []
        for start_node in nodes.keys():
            if start_node in adjacency:
                for end_node in adjacency[start_node]:
                    if end_node in nodes:
                        paths.append((start_node, end_node))
        
        if paths:
            print(f"✅ Found {len(paths)} direct connections:")
            for start, end in paths:
                print(f"  {start} → {end}")
        else:
            print("📭 No direct connections found")
    
    def find_shortest_path(self, nodes, edges):
        """Find shortest path between two nodes."""
        start_node = self.get_int_input("Enter start node ID")
        end_node = self.get_int_input("Enter end node ID")
        
        print(f"🔍 Finding shortest path from {start_node} to {end_node}...")
        
        # Simple BFS for shortest path
        if start_node not in nodes or end_node not in nodes:
            print("❌ One or both nodes not found")
            return
        
        # Build adjacency list
        adjacency = {}
        for edge_id, edge_data in edges.items():
            start = edge_data.get('start_node')
            end = edge_data.get('end_node')
            if start not in adjacency:
                adjacency[start] = []
            if end not in adjacency:
                adjacency[end] = []
            adjacency[start].append(end)
            adjacency[end].append(start)
        
        # BFS for shortest path
        queue = [(start_node, [start_node])]
        visited = set()
        
        while queue:
            current, path = queue.pop(0)
            if current == end_node:
                print(f"✅ Shortest path found: {' → '.join(map(str, path))}")
                return
            
            if current in visited:
                continue
            visited.add(current)
            
            if current in adjacency:
                for neighbor in adjacency[current]:
                    if neighbor not in visited:
                        queue.append((neighbor, path + [neighbor]))
        
        print("❌ No path found between the specified nodes")
    
    def find_paths_from_node(self, nodes, edges):
        """Find all paths from a specific node."""
        start_node = self.get_int_input("Enter start node ID")
        
        if start_node not in nodes:
            print(f"❌ Node {start_node} not found")
            return
        
        print(f"🔍 Finding all paths from node {start_node}...")
        
        # Build adjacency list
        adjacency = {}
        for edge_id, edge_data in edges.items():
            start = edge_data.get('start_node')
            end = edge_data.get('end_node')
            if start not in adjacency:
                adjacency[start] = []
            if end not in adjacency:
                adjacency[end] = []
            adjacency[start].append(end)
            adjacency[end].append(start)
        
        if start_node in adjacency:
            destinations = adjacency[start_node]
            print(f"✅ Node {start_node} can reach {len(destinations)} nodes:")
            for dest in destinations:
                print(f"  {start_node} → {dest}")
        else:
            print(f"❌ Node {start_node} has no connections")
    
    def publish_all_nodes(self):
        """Publish all nodes from internal storage."""
        print("🚀 Publishing all nodes from internal storage...")
        try:
            index = self.navigator.publish_all_nodes()
            if index > 0:
                print(f"✅ Successfully published nodes with index: {index}")
            else:
                print("⚠️ No nodes to publish")
        except Exception as e:
            print(f"❌ Error publishing nodes: {e}")
    
    def publish_all_edges(self):
        """Publish all edges from internal storage."""
        print("🚀 Publishing all edges from internal storage...")
        try:
            index = self.navigator.publish_all_edges()
            if index > 0:
                print(f"✅ Successfully published edges with index: {index}")
            else:
                print("⚠️ No edges to publish")
        except Exception as e:
            print(f"❌ Error publishing edges: {e}")
    
    def publish_all_nodes_and_edges(self):
        """Publish all nodes and edges from internal storage."""
        print("🚀 Publishing all nodes and edges from internal storage...")
        try:
            node_index, edge_index = self.navigator.publish_all_nodes_and_edges()
            print(f"✅ Successfully published nodes and edges")
            print(f"   📍 Node index: {node_index}")
            print(f"   🔗 Edge index: {edge_index}")
        except Exception as e:
            print(f"❌ Error publishing nodes and edges: {e}")
    
    def clear_internal_storage(self):
        """Clear all internal storage."""
        print("🗑️ Clearing internal storage...")
        try:
            self.navigator.clear_internal_storage()
            print("✅ Internal storage cleared successfully")
        except Exception as e:
            print(f"❌ Error clearing internal storage: {e}")
    
    def get_internal_storage_info(self):
        """Get information about internal storage."""
        print("📊 Internal Storage Information")
        print("=" * 35)
        try:
            info = self.navigator.get_internal_storage_info()
            print(f"📍 Nodes in storage: {info['nodes_count']}")
            print(f"🔗 Edges in storage: {info['edges_count']}")
            
            if info['nodes_count'] > 0:
                print(f"📍 Node IDs: {info['nodes']}")
            
            if info['edges_count'] > 0:
                print(f"🔗 Edge IDs: {info['edges']}")
            
            if info['nodes_count'] == 0 and info['edges_count'] == 0:
                print("📭 Internal storage is empty")
                
        except Exception as e:
            print(f"❌ Error getting internal storage info: {e}")
    
    def auto_collect_node(self):
        """Auto collect node at current pose (Demo B2 style)."""
        print("📍 Auto collecting node at current pose...")
        try:
            success, _ = self.navigator.add_node_at_current_pose_auto_collect(auto_connect=True)
            if success:
                print("✅ Node auto-collected successfully")
            else:
                print("❌ Failed to auto-collect node")
        except Exception as e:
            print(f"❌ Error auto-collecting node: {e}")
    
    def collect_and_save_nodes_edges(self):
        """Collect and save all nodes and edges (Demo B2 style)."""
        print("💾 Collecting and saving all nodes and edges...")
        try:
            node_idx, edge_idx = self.navigator.collect_and_save_nodes_edges(clear_after_save=True)
            if node_idx > 0 or edge_idx > 0:
                print(f"✅ Successfully saved nodes and edges")
                print(f"   📍 Node index: {node_idx}")
                print(f"   🔗 Edge index: {edge_idx}")
            else:
                print("⚠️ No edges to save")
        except Exception as e:
            print(f"❌ Error collecting and saving: {e}")
    
    def auto_collect_loop(self):
        """Start auto collection loop (Demo B2 style)."""
        print("🔄 Starting auto collection loop...")
        try:
            interval = self.get_float_input("Enter collection interval (seconds)", 2.0)
            max_nodes = self.get_int_input("Enter maximum nodes to collect", 10)
            
            print(f"🔄 Auto collection loop: {interval}s interval, max {max_nodes} nodes")
            print("Press Ctrl+C to stop collection")
            
            self.navigator.auto_collect_loop(node_interval=interval, max_nodes=max_nodes)
            
        except KeyboardInterrupt:
            print("⏹️ Auto collection stopped by user")
        except Exception as e:
            print(f"❌ Error in auto collection loop: {e}")
    
    def prepare_for_collection(self):
        """Prepare for node/edge collection (Demo B2 style)."""
        print("🔄 Preparing for node/edge collection...")
        try:
            self.navigator.prepare_for_collection()
            print("✅ Preparation completed")
        except Exception as e:
            print(f"❌ Error preparing for collection: {e}")
    
    def clear_and_start_mapping(self):
        """Clear all nodes/edges and start mapping (Demo B2 style)."""
        print("🗑️ Clearing all nodes and edges...")
        try:
            self.navigator.clear_collection_and_start_mapping()
            print("✅ Cleared and started mapping")
        except Exception as e:
            print(f"❌ Error clearing and starting mapping: {e}")
    
    def get_collection_status(self):
        """Get collection status (Demo B2 style)."""
        print("📊 Collection Status")
        print("=" * 20)
        try:
            status = self.navigator.get_collection_status()
            print(f"📍 Nodes collected: {status['nodes_collected']}")
            print(f"🔗 Edges collected: {status['edges_collected']}")
            
            if status['nodes_collected'] > 0:
                print(f"📍 Node IDs: {status['node_ids']}")
                print(f"📍 Last node: {status['last_node']}")
            
            if status['edges_collected'] > 0:
                print(f"🔗 Edge IDs: {status['edge_ids']}")
                print(f"🔗 Last edge: {status['last_edge']}")
            
            if status['nodes_collected'] == 0 and status['edges_collected'] == 0:
                print("📭 No nodes or edges collected yet")
                
        except Exception as e:
            print(f"❌ Error getting collection status: {e}")
    
    def get_nav_state(self):
        """Get navigation state."""
        print("📊 Getting navigation state...")
        
        try:
            nav_state = self.navigator.get_nav_state()
            
            print("📊 Navigation State:")
            print(f"  Mapping: {'✅ Active' if nav_state.get('mapping', False) else '❌ Inactive'}")
            print(f"  Navigation: {'✅ Active' if nav_state.get('navigation', False) else '❌ Inactive'}")
            print(f"  Relocation: {'✅ Active' if nav_state.get('relocation', False) else '❌ Inactive'}")
            print(f"  Pose initialized: {'✅ Yes' if nav_state.get('pose_initialized', False) else '❌ No'}")
            
            # Show additional state info if available
            for key, value in nav_state.items():
                if key not in ['mapping', 'navigation', 'relocation', 'pose_initialized']:
                    print(f"  {key}: {value}")
                    
        except Exception as e:
            print(f"❌ Error getting navigation state: {e}")
    
    def add_node_manual(self):
        """Add node with manual coordinates."""
        print("📍 Add Node (Manual)")
        print("=" * 20)
        
        node_name = self.get_int_input("Enter node name/ID", self.node_counter)
        x = self.get_float_input("Enter X coordinate", 0.0)
        y = self.get_float_input("Enter Y coordinate", 0.0)
        z = self.get_float_input("Enter Z coordinate", 0.0)
        yaw = self.get_float_input("Enter Yaw angle (radians)", 1.57)
        
        print(f"📍 Adding node {node_name} at ({x:.2f}, {y:.2f}, {z:.2f}), yaw: {yaw:.2f}")
        self.navigator.add_node(node_name, x, y, z, yaw)
        print("✅ Node added successfully")
        self.node_counter += 1
    
    def show_help(self):
        """Show help information."""
        print("\n🎮 Interactive Navigator Help")
        print("=" * 50)
        print("📋 Available commands:")
        print("  [M] Start/End Mapping")
        print("  [N] Add Node at Current Pose")
        print("  [NM] Add Node (Manual coordinates)")
        print("  [E] Add Edge between Nodes")
        print("  [CN] Close All Nodes")
        print("  [S] Start Navigation")
        print("  [P] Pause Navigation")
        print("  [R] Recover Navigation")
        print("  [I] Initialize Pose")
        print("  [L] Start Relocation and Start Collect node and edge data")
        print("  [FP] Fixed Point Navigation")
        print("  [NS] Get Navigation State")
        print("\n🗑️ Data Management:")
        print("  [C] Clear Environment Cloud")
        print("  [T] Clear Trajectory Cloud")
        print("  [A] Clear All Clouds")
        print("  [V] Save Environment Cloud")
        print("  [R] Save Trajectory Cloud")
        print("  [S] Save Combined Cloud")
        print("\n📍 Pose & Status:")
        print("  [G] Get Current Pose")
        print("  [RT] Check Realtime Pose Status")
        print("  [I] Get Cloud Info")
        print("  [D] Set Downsample Parameters")
        print("\n🗑️ Node/Edge Management:")
        print("  [DN] Delete Nodes")
        print("  [DE] Delete Edges")
        print("  [DAN] Delete All Nodes")
        print("  [DAE] Delete All Edges")
        print("  [QN] Query Nodes")
        print("  [QE] Query Edges")
        print("  [NO] Network Overview (Complete network analysis)")
        print("  [PA] Path Analysis (Find paths between nodes)")
        print("\n💾 Internal Storage Management:")
        print("  [PN] Publish All Nodes (from internal storage)")
        print("  [PE] Publish All Edges (from internal storage)")
        print("  [PNE] Publish All Nodes and Edges (from internal storage)")
        print("  [CI] Clear Internal Storage")
        print("  [GI] Get Internal Storage Info")
        print("\n🔄 Auto Collection (Demo B2 Style):")
        print("  [AC] Auto Collect Node at Current Pose")
        print("  [CS] Collect and Save All Nodes/Edges")
        print("  [AL] Auto Collect Loop")
        print("  [PC] Prepare for Collection")
        print("  [CM] Clear and Start Mapping")
        print("  [GS] Get Collection Status")
        print("\n📷 Camera Control:")
        print("  [CA] Camera Control Menu")
        print("\n🎬 Visualization Control:")
        print("  [VS] Start Visualization")
        print("  [VT] Stop Visualization")
        print("  [VH] Check Visualization Status")
        print("\n❓ Help & Quit:")
        print("  [H] Show Help")
        print("  [Q] Quit")
        print("=" * 50)
    
    def process_command(self, command: str):
        """Process user command."""
        command = command.upper().strip()
        
        if command == 'M':
            # Toggle mapping
            choice = input("Start (S) or End (E) mapping? ").upper().strip()
            if choice == 'S':
                self.start_mapping()
            elif choice == 'E':
                self.end_mapping()
            else:
                print("Invalid choice")
        
        elif command == 'N':
            self.add_node_at_current_pose()
        
        elif command == 'E':
            self.add_edge()
        
        elif command == 'CN':
            self.close_all_nodes()
        
        elif command == 'S':
            self.start_relocation()
            self.start_navigation()
            self.initialize_pose()
            self.start_navigation_loop()

        
        elif command == 'P':
            self.pause_navigation()
        
        elif command == 'R':
            self.recover_navigation()
        
        elif command == 'I':
            self.initialize_pose()
        
        elif command == 'L':
            self.delete_all_nodes()
            self.delete_all_edges()
            self.start_relocation()
            self.start_navigation()
            self.initialize_pose()

        
        elif command == 'C':
            self.clear_cloud()
        
        elif command == 'T':
            self.clear_trajectory()
        
        elif command == 'A':
            self.clear_all_clouds()
        
        elif command == 'V':
            self.save_cloud()
        
        elif command == 'R':
            self.save_trajectory()
        
        elif command == 'S':
            self.save_combined()
        
        elif command == 'G':
            self.get_current_pose()
        
        elif command == 'I':
            self.get_cloud_info()
        
        elif command == 'D':
            self.set_downsample_params()
        
        elif command == 'NR':
            self.add_edge_with_restriction()
        
        elif command == 'ER':
            self.add_edge_with_restriction()
        
        elif command == 'VS':
            self.start_visualization()
        
        elif command == 'VT':
            self.stop_visualization()
        
        elif command == 'VH':
            self.check_visualization_status()
        
        elif command == 'FP':
            self.fixed_point_navigation()
        
        elif command == 'CA':
            self.camera_control()
        
        elif command == 'DN':
            self.delete_node()
        
        elif command == 'DE':
            self.delete_edge()
        
        elif command == 'DAN':
            self.delete_all_nodes()
        
        elif command == 'DAE':
            self.delete_all_edges()
        
        elif command == 'QN':
            self.query_nodes()
        
        elif command == 'QE':
            self.query_edges()
        
        elif command == 'NO':
            self.query_network_overview()
        
        elif command == 'PA':
            self.query_path_analysis()
        
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
        
        elif command == 'NS':
            self.get_nav_state()
        
        elif command == 'NM':
            self.add_node_manual()
        
        elif command == 'RT':
            self.check_realtime_pose_status()
        
        elif command == 'H':
            self.show_help()
        
        elif command == 'Q':
            print("👋 Goodbye!")
            self.running = False
        
        else:
            print("❌ Unknown command. Type 'H' for help.")
    
    def run(self):
        """Main run loop."""
        try:
            # 主线程里初始化可视化窗口
            # self.navigator.start_visualization()
            
            # 创建后台线程处理ROS消息
            def spin_thread():
                while self.running:
                    rclpy.spin_once(self.navigator, timeout_sec=0.1)
            
            # 启动ROS消息处理线程
            spin_thread = threading.Thread(target=spin_thread, daemon=True)
            spin_thread.start()
            
            # 主线程处理用户输入
            while self.running:
                try:
                    command = input("\nEnter command: ").strip()
                    if command:
                        self.process_command(command)
                        print()  # Add blank line for readability
                except EOFError:
                    print("\n⏹️ End of input")
                    break
                
        except KeyboardInterrupt:
            print("\n⏹️ Interrupted by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources."""
        print("🧹 Cleaning up...")
        self.navigator.stop_visualization()
        self.navigator.destroy_node()
        rclpy.shutdown()


def main():
    """Main function."""
    print("🎮 Starting Interactive Navigator...")
    
    # Create and run interactive navigator
    interactive_nav = InteractiveNavigator()
    interactive_nav.run()


if __name__ == "__main__":
    main()
