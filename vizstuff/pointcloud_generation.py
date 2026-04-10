import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
import cv2

def pointcloud_to_depth_image(pc_reordered):
    """
    Convert the reordered point cloud (H, W, 3) into a depth image.
    White = closer, Black = farther.
    """

    # Extract Z from (X, Y, Z)
    Z = pc_reordered[:, :, 2]

    # Clean invalid values
    Z = np.where(np.isfinite(Z), Z, np.nan)

    # Compute min/max ignoring NaN
    z_min = np.nanmin(Z)
    z_max = np.nanmax(Z)

    # Prevent divide-by-zero
    if z_max == z_min:
        z_max += 1e-6

    # Normalize depth: 0=far (black), 255=near (white)
    depth_norm = (z_max - Z) / (z_max - z_min)  # invert so near = white
    depth_norm = np.clip(depth_norm, 0, 1)

    # Convert to 8-bit grayscale image
    depth_img = (depth_norm * 255).astype(np.uint8)

    return depth_img

def process_disparity(left_image, right_image):
    """
    left_image, right_image : uint8 numpy arrays (H,W)
    Returns: point_cloud (H, W, 3) float32
             matching_time, filtering_time
    """

    # ---- Parameters matching Unity ----
    algo = "bm"
    filter_type = "wls_conf"
    no_display = True
    no_downscale = True
    max_disp = 160
    lambda_val = 8000.0
    sigma = 1.5
    vis_mult = 1.0

    height, width = left_image.shape

    # ---- Load calibration identical to Unity ----
    fs = cv2.FileStorage("../calibration_params/stereocalib_params.xml", cv2.FILE_STORAGE_READ)
    cameraMatrix1 = fs.getNode("cameraMatrix1").mat()
    cameraMatrix2 = fs.getNode("cameraMatrix2").mat()
    distCoeffs1 = fs.getNode("distCoeffs1").mat()
    distCoeffs2 = fs.getNode("distCoeffs2").mat()
    R = fs.getNode("rotationVectors").mat()
    T = fs.getNode("translationVectors").mat()
    fs.release()

    # ---- Stereo Rectify ----
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        cameraMatrix1, distCoeffs1,
        cameraMatrix2, distCoeffs2,
        (width, height), R, T
    )

    map11, map12 = cv2.initUndistortRectifyMap(
        cameraMatrix1, distCoeffs1, R1, P1,
        (width, height), cv2.CV_16SC2
    )
    map21, map22 = cv2.initUndistortRectifyMap(
        cameraMatrix2, distCoeffs2, R2, P2,
        (width, height), cv2.CV_16SC2
    )

    rect_left = cv2.remap(left_image, map11, map12, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_image, map21, map22, cv2.INTER_LINEAR)

    # ---- Downscale logic (Unity disables) ----
    if not no_downscale:
        max_disp //= 2
        if max_disp % 16 != 0:
            max_disp += 16 - max_disp % 16
        left_m = cv2.resize(rect_left, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR_EXACT)
        right_m = cv2.resize(rect_right, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR_EXACT)
    else:
        left_m = rect_left.copy()
        right_m = rect_right.copy()

    # ---- StereoBM + WLS Matching ----
    wsize = 15  # Unity default for no-downscale BM

    left_matcher = cv2.StereoBM_create(numDisparities=max_disp, blockSize=wsize)
    wls = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    # Matching
    t = cv2.getTickCount()
    disp_left = left_matcher.compute(left_m, right_m)
    disp_right = right_matcher.compute(right_m, left_m)
    matching_time = (cv2.getTickCount() - t) / cv2.getTickFrequency()

    # ---- WLS Filtering ----
    wls.setLambda(lambda_val)
    wls.setSigmaColor(sigma)

    t = cv2.getTickCount()
    disp_filtered = wls.filter(disp_left, rect_left, None, disp_right)
    filtering_time = (cv2.getTickCount() - t) / cv2.getTickFrequency()

    # ---- Get Visualization Disparity (Unity uses this!) ----
    disp_vis = cv2.ximgproc.getDisparityVis(disp_filtered, vis_mult).astype(np.float32)

    # ---- Reproject Using disp_vis (Unity bug—but we match it) ----
    point_cloud = cv2.reprojectImageTo3D(disp_vis, Q, handleMissingValues=True)

    # ---- Match Unity coordinate reorder (Y, Z, X) ----
    pc_reordered = np.zeros_like(point_cloud)
    pc_reordered[:, :, 0] = point_cloud[:, :, 1]  # Y → X
    pc_reordered[:, :, 1] = point_cloud[:, :, 2]  # Z → Y
    pc_reordered[:, :, 2] = point_cloud[:, :, 0]  # X → Z

    return pc_reordered, matching_time, filtering_time



# --- Load disparity map ---
# Using YML
fs = cv2.FileStorage("disp.yml", cv2.FILE_STORAGE_READ)
disparity = fs.getNode("disp").mat().astype(np.float32)
fs.release()

# If StereoBM/SGBM, divide by 16
# disparity /= 16.0

disparity = disparity.astype(np.float32)

fs = cv2.FileStorage("Q_matrix.yml", cv2.FILE_STORAGE_READ)
Q = fs.getNode("Q").mat()
fs.release()

print("Loaded Q:\n", Q)

point_cloud = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=False)

# ---- Match Unity coordinate reorder (Y, Z, X) ----
pc_reordered = np.zeros_like(point_cloud)
pc_reordered[:, :, 0] = point_cloud[:, :, 1]  # Y → X
pc_reordered[:, :, 1] = point_cloud[:, :, 2]  # Z → Y
pc_reordered[:, :, 2] = point_cloud[:, :, 0]  # X → Z

depth_image = pointcloud_to_depth_image(pc_reordered)

cv2.imshow("Depth Image", depth_image)
cv2.waitKey(0)

# --- Reproject disparity to 3D ---
points_3D = cv2.reprojectImageTo3D(disparity, Q)
mask = disparity > 0
points_3D = points_3D[mask]

# ---- Inputs you must set to match Unity ----
thermal_height = 62
thermal_width = 80

stereo_height = disparity.shape[0]
stereo_width = disparity.shape[1]

stereo_HFOV = 62.2
stereo_VFOV = 48.8
thermal_HFOV = 45
thermal_VFOV = 45


# Load thermal image
thermal = np.load('image-20-00-03-6_data.npy').astype(np.float32)
thermal = thermal.reshape((thermal_height, thermal_width))

# --- Compute mapping rectangle ---
horizontal_fill = thermal_HFOV / stereo_HFOV * stereo_width
vertical_fill   = thermal_VFOV / stereo_VFOV * stereo_height

thermal_y_min = int((stereo_height - vertical_fill) / 2)
thermal_y_max = stereo_height - thermal_y_min

thermal_x_min = int((stereo_width - horizontal_fill) / 2)
thermal_x_max = stereo_width - thermal_x_min

# Prepare thermal values for all stereo pixels
thermal_map = np.full((stereo_height, stereo_width), -1, dtype=np.float32)

for y in range(stereo_height):
    for x in range(stereo_width):

        if (thermal_y_min <= y <= thermal_y_max and
            thermal_x_min <= x <= thermal_x_max):

            relX = (x - thermal_x_min) / (thermal_x_max - thermal_x_min)
            relY = (y - thermal_y_min) / (thermal_y_max - thermal_y_min)

            tX = int(round(relX * (thermal_width - 1)))
            tY = int(round(relY * (thermal_height - 1)))

            thermal_map[y, x] = thermal[tY, tX]
        else:
            thermal_map[y, x] = -1

# --- Create PyVista point cloud ---
# cloud = pv.PolyData(points_3D)
# cloud['colors'] = (thermal_colors * 255).astype(np.uint8)  # PyVista expects 0-255
#
# # --- Visualize ---
# plotter = pv.Plotter()
# plotter.add_points(cloud, scalars='colors', rgb=True, point_size=2)
#
# plotter.reset_camera()
# plotter.enable_parallel_projection()
#
# c = cloud.center
# d = cloud.length
#
# # Look from NEGATIVE Z (correct for stereo)
# plotter.camera.position = (c[0], c[1], c[2] - d * 2)
# plotter.camera.focal_point = c
# plotter.camera.up = (0, -1, 0)
#
# plotter.show()
# x = points_3D[:, 0]
# y = points_3D[:, 1]
#
# plt.figure(figsize=(8,6))
# plt.scatter(x, y, c=thermal_colors, s=1)
# plt.axis('equal')
# plt.show()

# points_3D -> depth
depth = points_3D[:, 2]
# initialize empty image
img_depth = np.zeros((stereo_height, stereo_width), dtype=np.float32)
mask_idx = np.where(mask)  # mask from disparity
img_depth[mask_idx] = depth

# normalize for display
img_disp = cv2.normalize(img_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
cv2.imshow("Depth", img_disp)
cv2.waitKey(0)
cv2.destroyAllWindows()

# depth = points_3D[:, 2]  # Z-coordinate
#
# # compute disparity from depth
# disparity = (focal_length_px * baseline_m) / depth
#
# # fill disparity image
# img_disp = np.zeros((stereo_height, stereo_width), dtype=np.float32)
# img_disp[mask_idx] = disparity
#
# # normalize for visualization
# img_disp_vis = cv2.normalize(img_disp, None, 0, 255, cv2.NORM_MINMAX)
# img_disp_vis = img_disp_vis.astype(np.uint8)
#
# cv2.imshow("Disparity", img_disp_vis)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# color_img = cv2.imread('image-19-44-07-1_image.png', cv2.IMREAD_COLOR)
# # OpenCV loads as BGR, convert to RGB
# color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
#
# # Resize color image to match disparity resolution
# color_resized = cv2.resize(color_img, (w, h), interpolation=cv2.INTER_LINEAR)
#
# # Apply mask to get colors corresponding to valid disparity points
# color_points = color_resized[mask]
#
# # --- Create PyVista point cloud ---
# cloud = pv.PolyData(points_3D)
# cloud['colors'] = color_points.astype(np.uint8)  # keep the PNG’s exact colors
#
# # --- Visualize ---
# plotter = pv.Plotter()
# plotter.add_points(cloud, scalars='colors', rgb=True, point_size=5)
# plotter.show()