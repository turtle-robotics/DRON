import open3d as o3d

# Load the PLY file
pcd = o3d.io.read_point_cloud("pointcloud.ply")

# Visualize it
o3d.visualization.draw_geometries([pcd])
