#!/usr/bin/env python3
"""
Interactive Navigator Example with Keyboard Controls

This script provides an interactive interface for controlling the Navigator class
using keyboard inputs for various navigation operations.
"""

import rclpy
import time
from navigator import Navigator


class InteractiveNavigator:
    """
    Interactive navigator with keyboard controls.
    """
    
    def __init__(self):
        """Initialize the interactive navigator."""
        rclpy.init()
        self.navigator = Navigator(enable_visualization=True)
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
        print("  [L] Start Relocation")
        print("  [C] Clear Environment Cloud")
        print("  [T] Clear Trajectory Cloud")
        print("  [A] Clear All Clouds")
        print("  [V] Save Environment Cloud")
        print("  [R] Save Trajectory Cloud")
        print("  [S] Save Combined Cloud")
        print("  [G] Get Current Pose")
        print("  [I] Get Cloud Info")
        print("  [D] Set Downsample Parameters")
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
        
        print("📍 Adding node at current pose...")
        if self.navigator.add_node_at_current_pose(node_name):
            print(f"✅ Node {node_name} added successfully")
            self.node_counter += 1
        else:
            print("❌ Failed to add node - no pose data available")
    
    def add_edge(self):
        """Add edge between nodes."""
        start_node = self.get_int_input("Enter start node ID", 1)
        end_node = self.get_int_input("Enter end node ID", 2)
        edge_name = self.get_int_input("Enter edge name/ID", self.edge_counter)
        speed = self.get_float_input("Enter dog speed", 1.0)
        
        print(f"🔗 Adding edge {edge_name} from node {start_node} to {end_node}...")
        self.navigator.add_edge(edge_name, start_node, end_node, speed)
        print("✅ Edge added successfully")
        self.edge_counter += 1
    
    def start_navigation(self):
        """Start navigation."""
        print("🚀 Starting navigation...")
        self.navigator.start_navigation()
        print("✅ Navigation started")
    
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
        x = self.get_float_input("Enter X position", 0.0)
        y = self.get_float_input("Enter Y position", 0.0)
        z = self.get_float_input("Enter Z position", 0.0)
        
        self.navigator.pose_init(translation=(x, y, z))
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
    
    def show_help(self):
        """Show help information."""
        print("\n🎮 Interactive Navigator Help")
        print("=" * 40)
        print("Available commands:")
        print("  [M] Start/End Mapping")
        print("  [N] Add Node at Current Pose")
        print("  [E] Add Edge between Nodes")
        print("  [S] Start Navigation")
        print("  [P] Pause Navigation")
        print("  [R] Recover Navigation")
        print("  [I] Initialize Pose")
        print("  [L] Start Relocation")
        print("  [C] Clear Environment Cloud")
        print("  [T] Clear Trajectory Cloud")
        print("  [A] Clear All Clouds")
        print("  [V] Save Environment Cloud")
        print("  [R] Save Trajectory Cloud")
        print("  [S] Save Combined Cloud")
        print("  [G] Get Current Pose")
        print("  [I] Get Cloud Info")
        print("  [D] Set Downsample Parameters")
        print("  [H] Show Help")
        print("  [Q] Quit")
        print("=" * 40)
    
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
        
        elif command == 'S':
            self.start_navigation()
        
        elif command == 'P':
            self.pause_navigation()
        
        elif command == 'R':
            self.recover_navigation()
        
        elif command == 'I':
            self.initialize_pose()
        
        elif command == 'L':
            self.start_relocation()
        
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
            self.navigator.start_visualization()
            while self.running:
                # Process ROS messages
                rclpy.spin_once(self.navigator, timeout_sec=0.1)
                
                # Get user command
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