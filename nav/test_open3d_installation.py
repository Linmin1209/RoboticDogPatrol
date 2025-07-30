#!/usr/bin/env python3
"""
Test script to check Open3D installation and basic functionality.
"""

def test_open3d_installation():
    """Test Open3D installation and basic functionality."""
    print("🧪 Testing Open3D Installation")
    print("=" * 40)
    
    try:
        # Test import
        print("📦 Testing Open3D import...")
        import open3d as o3d
        print(f"✅ Open3D version: {o3d.__version__}")
        
        # Test basic functionality
        print("🔧 Testing basic Open3D functionality...")
        
        # Create a simple point cloud
        pcd = o3d.geometry.PointCloud()
        import numpy as np
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([1, 0, 0])  # Red color
        
        print(f"✅ Created point cloud with {len(pcd.points)} points")
        
        # Test visualization (optional)
        print("🎨 Testing visualization...")
        try:
            vis = o3d.visualization.Visualizer()
            if vis is None:
                print("❌ Failed to create Visualizer")
                return False
                
            window_created = vis.create_window("Test Window", width=800, height=600)
            if not window_created:
                print("❌ Failed to create visualization window")
                return False
                
            # Add geometry
            vis.add_geometry(pcd)
            
            # Update
            vis.poll_events()
            vis.update_renderer()
            
            print("✅ Visualization test passed")
            
            # Close window
            vis.destroy_window()
            
        except Exception as e:
            print(f"❌ Visualization test failed: {e}")
            return False
        
        print("✅ All Open3D tests passed!")
        return True
        
    except ImportError as e:
        print(f"❌ Open3D import failed: {e}")
        print("💡 Install Open3D with: pip install open3d")
        return False
    except Exception as e:
        print(f"❌ Open3D test failed: {e}")
        return False


if __name__ == "__main__":
    success = test_open3d_installation()
    if success:
        print("\n🎉 Open3D is ready for use!")
    else:
        print("\n⚠️ Open3D needs to be fixed before using visualization features.") 