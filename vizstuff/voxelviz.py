import random

import numpy as np
import open3d as o3d
import cv2

def visualize_voxels(points, voxel_size=0.1):
    """
    Visualize a list of 3D points as voxels.

    Parameters
    ----------
    points : list or np.ndarray
        Nx3 array of XYZ coordinates
    voxel_size : float
        Size of each voxel cube
    """

    # Convert to numpy array
    points = np.array(points)

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Convert point cloud to voxel grid
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
        pcd,
        voxel_size=voxel_size
    )

    # Visualize
    o3d.visualization.draw_geometries([voxel_grid])

def disparity_to_points(disparity_map, focal_length, baseline, cx=None, cy=None):
    """
    Convert a disparity map into a list of 3D points.

    Parameters
    ----------
    disparity_map : np.ndarray
        HxW disparity image (pixels)
    focal_length : float
        Camera focal length in pixels
    baseline : float
        Distance between stereo cameras (meters)
    cx, cy : float
        Principal point (defaults to image center)

    Returns
    -------
    np.ndarray
        Nx3 array of XYZ points
    """

    h, w = disparity_map.shape

    if cx is None:
        cx = w / 2
    if cy is None:
        cy = h / 2

    u, v = np.meshgrid(np.arange(w), np.arange(h))

    d = disparity_map.astype(np.float32)

    valid = d > 0

    Z = np.zeros_like(d)
    Z[valid] = (focal_length * baseline) / d[valid]

    X = (u - cx) * Z / focal_length
    Y = (v - cy) * Z / focal_length

    points = np.stack((X, Y, Z), axis=-1)

    return points[valid]

if __name__ == "__main__":
    # Example: random points

    # disparity = cv2.imread("filtered_disparity.png", cv2.IMREAD_UNCHANGED)
    #
    # points = disparity_to_points(
    #     disparity,
    #     focal_length=700,
    #     baseline=1
    # )

    pcd = o3d.io.read_point_cloud("point_cloud.ply")

    voxel_size = .3

    # Create voxel grid
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

    voxels = voxel_grid.get_voxels()
    origin = voxel_grid.origin

    points = []
    colors = []

    for v in voxels:
        # convert grid index to voxel center
        center = origin + (np.array(v.grid_index) + 0.5) * voxel_size
        points.append(center)

        # blue-ish random noise
        color = [0,0,np.random.normal(.7,.08)]

        colors.append(np.clip(color, 0, 1))

    points = np.array(points)
    colors = np.array(colors)

    # create colored point cloud
    colored_pcd = o3d.geometry.PointCloud()
    colored_pcd.points = o3d.utility.Vector3dVector(points)
    colored_pcd.colors = o3d.utility.Vector3dVector(colors)

    # convert back to voxel grid (preserves color)
    colored_voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
        colored_pcd, voxel_size
    )

    o3d.visualization.draw_geometries([colored_voxel_grid])
