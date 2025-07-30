#!/usr/bin/env python3
"""
Test script for fixed visualization without Open3D warnings.
"""

import rclpy
import time
from navigator import Navigator


def test_fixed_visualization():
    """Test the fixed visualization functionality."""
    print("🧪 Testing Fixed Visualization (No Warnings)")
    print("=" * 50)
    
    rclpy.init()
    
    # Test with visualization enabled
    print("🚀 Testing with visualization enabled...")
    navigator = Navigator(enable_visualization=True)
    
    try:
        # Start visualization
        navigator.start_visualization()
        
        # Test cloud size monitoring
        print("📊 Testing cloud size monitoring...")
        env_size = navigator.get_cloud_size()
        traj_size = navigator.get_trajectory_size()
        total_size = navigator.get_total_cloud_size()
        print(f"   Environment cloud size: {env_size}")
        print(f"   Trajectory cloud size: {traj_size}")
        print(f"   Total cloud size: {total_size}")
        
        # Wait for some time to see if visualization works
        print("⏳ Waiting for visualization (10 seconds)...")
        print("   Check if the visualization window appears without warnings")
        
        for i in range(10):
            env_size = navigator.get_cloud_size()
            traj_size = navigator.get_trajectory_size()
            total_size = navigator.get_total_cloud_size()
            print(f"   Time {i+1}/10: Env={env_size}, Traj={traj_size}, Total={total_size}")
            rclpy.spin_once(navigator, timeout_sec=1.0)
        
        # Test cloud info
        print("📊 Cloud information:")
        navigator.get_cloud_info()
        
        print("✅ Fixed visualization test completed!")
        
    except KeyboardInterrupt:
        print("⏹️ Test interrupted by user")
    except Exception as e:
        print(f"❌ Test failed: {e}")
    finally:
        navigator.stop_visualization()
        navigator.destroy_node()
    
    # Test with visualization disabled
    print("\n🚀 Testing with visualization disabled...")
    navigator_no_vis = Navigator(enable_visualization=False)
    
    try:
        # Test cloud size monitoring
        print("📊 Testing cloud size monitoring (no visualization)...")
        env_size = navigator_no_vis.get_cloud_size()
        traj_size = navigator_no_vis.get_trajectory_size()
        total_size = navigator_no_vis.get_total_cloud_size()
        print(f"   Environment cloud size: {env_size}")
        print(f"   Trajectory cloud size: {traj_size}")
        print(f"   Total cloud size: {total_size}")
        
        print("✅ No visualization test completed!")
        
    except Exception as e:
        print(f"❌ No visualization test failed: {e}")
    finally:
        navigator_no_vis.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    test_fixed_visualization() 