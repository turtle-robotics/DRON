import trimesh
import numpy as np
import open3d as o3d

# Load the mesh file
mesh = trimesh.load('obj/10042_Sea_Turtle_V2_iterations-2.obj')

# If the file loads as a Scene, combine meshes
if isinstance(mesh, trimesh.Scene):
    mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))


# Remove degenerate faces
# mesh.update_faces(mesh.nondegenerate_faces())

# # Remove duplicate faces
# mesh.update_faces(mesh.unique_faces())
#
# # Remove unused vertices
# mesh.remove_unreferenced_vertices()

# Number of points to sample
num_points = 100000

# Sample points on the surface
points, face_indices = trimesh.sample.sample_surface(mesh, num_points)

normals = mesh.face_normals[face_indices]
noise_strength = 0.9

points += normals * np.random.normal(scale=noise_strength, size=(len(points), 1))

# Convert to Open3D point cloud
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# Visualize
o3d.visualization.draw_geometries([point_cloud])

# Save
o3d.io.write_point_cloud("point_cloud.ply", point_cloud)

print("Point cloud saved as point_cloud.ply")