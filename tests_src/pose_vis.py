import numpy as np
import open3d as o3d
import time

def read_tum_trajectory_file(filename):
    """Read the TUM format trajectory file and return timestamps, positions, and quaternions."""
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:8]  # qx, qy, qz, qw
    return timestamps, positions, quaternions

def quaternion_to_rotation_matrix(quaternion):
    """Convert quaternion [qx, qy, qz, qw] to rotation matrix."""
    qx, qy, qz, qw = quaternion
    
    rotation_matrix = np.array([
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
    ])
    
    return rotation_matrix

def create_robot_cube(size=0.5, color=[1, 0, 0]):
    """Create a cube to represent the robot."""
    cube = o3d.geometry.TriangleMesh.create_box(width=size, height=size, depth=size)
    cube.compute_vertex_normals()
    cube.translate([-size/2, -size/2, -size/2])  # Center the cube at origin
    cube.paint_uniform_color(color)
    return cube

def create_scale_reference(length=10, height=0.1):
    """Create a scale reference bar with 1-meter increments."""
    # Create main scale line
    points = []
    lines = []
    colors = []
    
    # Main horizontal line
    points.append([0, 0, 0])
    points.append([length, 0, 0])
    lines.append([0, 1])
    colors.append([0, 0, 0])  # Black color
    
    # Add tick marks for each meter
    for i in range(length + 1):
        # Vertical tick
        points.append([i, 0, 0])
        points.append([i, height, 0])
        lines.append([len(points)-2, len(points)-1])
        colors.append([0, 0, 0])
    
    scale = o3d.geometry.LineSet()
    scale.points = o3d.utility.Vector3dVector(points)
    scale.lines = o3d.utility.Vector2iVector(lines)
    scale.colors = o3d.utility.Vector3dVector(colors)
    
    return scale

def visualize_trajectory(timestamps, positions, quaternions, skip_frames=10):
    """Visualize the robot trajectory."""
    
    # Create trajectory line set
    trajectory_points = o3d.utility.Vector3dVector(positions)
    trajectory_lines = []
    for i in range(len(positions) - 1):
        trajectory_lines.append([i, i + 1])
    
    trajectory_line_set = o3d.geometry.LineSet()
    trajectory_line_set.points = trajectory_points
    trajectory_line_set.lines = o3d.utility.Vector2iVector(trajectory_lines)
    trajectory_line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0] for _ in range(len(trajectory_lines))])
    
    # Create scale reference
    scale_reference = create_scale_reference()
    
    # Get bounds of trajectory to position the scale
    trajectory_min = np.min(positions, axis=0)
    trajectory_max = np.max(positions, axis=0)
    
    # Position the scale in the bottom-left of the visualization area
    scale_position = [trajectory_min[0], trajectory_min[1], trajectory_max[2]]
    scale_reference.translate(scale_position)
    
    # Set up custom visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1024, height=768)
    
    # Add trajectory
    vis.add_geometry(trajectory_line_set)
    vis.add_geometry(scale_reference)
    
    # Set initial view to bird's eye
    view_control = vis.get_view_control()
    view_control.set_front([0, 0, -1])  # Looking down the z-axis
    view_control.set_up([0, 1, 0])      # Y-axis is up
    view_control.set_lookat([np.mean(positions[:, 0]), np.mean(positions[:, 1]), np.mean(positions[:, 2])])
    view_control.set_zoom(0.8)
    
    # Animation callback
    cube = create_robot_cube()
    vis.add_geometry(cube)
    
    for i in range(0, len(positions), skip_frames):
        position = positions[i]
        quaternion = quaternions[i]
        
        # Create transformation matrix
        transformation = np.eye(4)
        transformation[:3, :3] = quaternion_to_rotation_matrix(quaternion)
        transformation[:3, 3] = position
        
        # Update cube position
        cube.clear()
        temp_cube = create_robot_cube()
        temp_cube.transform(transformation)
        cube.vertices = temp_cube.vertices
        cube.triangles = temp_cube.triangles
        cube.vertex_normals = temp_cube.vertex_normals
        cube.triangle_normals = temp_cube.triangle_normals
        cube.vertex_colors = temp_cube.vertex_colors
        
        vis.update_geometry(cube)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.05)  # Control animation speed
    
    # vis.destroy_window()

if __name__ == "__main__":
    # Read the trajectory file
    file_path = "M3ED/spot_forest_easy_1_pose_evo_gt.txt"
    timestamps, positions, quaternions = read_tum_trajectory_file(file_path)
    
    # Visualize the trajectory
    visualize_trajectory(timestamps, positions, quaternions)