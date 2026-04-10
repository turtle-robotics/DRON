import numpy as np
import cv2

# Load disparity map (assume it's already computed)
disparity = cv2.imread('filtered_disparity.png', cv2.IMREAD_UNCHANGED).astype(np.float32)

# Convert disparity if needed (StereoSGBM often multiplies by 16)
disparity /= 16.0

# Camera parameters (example values)
focal_length = 718.8560  # in pixels
baseline = 0.54           # in meters
cx, cy = 607.1928, 185.2157  # principal point

# Create Q matrix
Q = np.float32([[1, 0, 0, -cx],
                [0, -1, 0, cy],
                [0, 0, 0, -focal_length],
                [0, 0, 1/baseline, 0]])

# Reproject to 3D
points_3D = cv2.reprojectImageTo3D(disparity, Q)

# Mask out points where disparity is zero
mask = disparity > 0
out_points = points_3D[mask]

# Save to a simple PLY without colors
def write_ply_no_color(filename, verts):
    verts = verts.reshape(-1, 3)
    ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
end_header
'''
    with open(filename, 'w') as f:
        f.write(ply_header % dict(vert_num=len(verts)))
        for v in verts:
            f.write('%f %f %f\n' % (v[0], v[1], v[2]))

write_ply_no_color('pointcloud.ply', out_points)
print("Saved point cloud to pointcloud.ply")
