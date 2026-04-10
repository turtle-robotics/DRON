import cv2
import numpy as np
import open3d as o3d
import cv2

def image_rectify(left_img, right_img):
    fs = cv2.FileStorage("stereocalib_params.xml", cv2.FILE_STORAGE_READ)

    cameraMatrix1 = fs.getNode("cameraMatrix1").mat()
    cameraMatrix2 = fs.getNode("cameraMatrix2").mat()

    distCoeffs1 = fs.getNode("distCoeffs1").mat()
    distCoeffs2 = fs.getNode("distCoeffs2").mat()

    R = fs.getNode("rotationVectors").mat()
    T = fs.getNode("translationVectors").mat()

    image_size = (3280, 2464)  # replace with your image resolution

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        cameraMatrix1,
        distCoeffs1,
        cameraMatrix2,
        distCoeffs2,
        image_size,
        R,
        T
    )

    print("R1: ", R1)
    print("R2: ", R2)

    mapLx, mapLy = cv2.initUndistortRectifyMap(
        cameraMatrix1, distCoeffs1, R1, P1, image_size, cv2.CV_32FC1
    )

    mapRx, mapRy = cv2.initUndistortRectifyMap(
        cameraMatrix2, distCoeffs2, R2, P2, image_size, cv2.CV_32FC1
    )

    rectL = cv2.remap(left_img, mapLx, mapLy, cv2.INTER_LINEAR)
    rectR = cv2.remap(right_img, mapRx, mapRy, cv2.INTER_LINEAR)

    cv2.imshow("rectL", rectL)
    cv2.waitKey(0)

    fs.release()

    return rectL, rectR,Q,P1,P2


def compute_disparity(left_img, right_img,P1,P2):
    """
    Compute disparity using StereoSGBM
    """

    left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 5,  # must be divisible by 16
        blockSize=10,
        # P1=P1,
        # P2=P2,
        # disp12MaxDiff=1,
        # uniquenessRatio=10,
        # speckleWindowSize=100,
        # speckleRange=32
    )

    disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

    cv2.imshow("disparity", disparity / disparity.max())
    cv2.waitKey(0)

    return disparity


def disparity_to_points(disparity, Q):
    """
    Convert disparity map into Nx3 point cloud
    """

    points_3d = cv2.reprojectImageTo3D(disparity, Q)

    mask = disparity > 0
    points = points_3d[mask]

    return points


def visualize_voxels(points, voxel_size=0.2):
    """
    Visualize points as voxels in Open3D
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
        pcd,
        voxel_size=voxel_size
    )

    o3d.visualization.draw_geometries([voxel_grid])


def main():

    # Load stereo images
    left = cv2.imread("f1left.jpg")
    right = cv2.imread("f1right.jpg")

    rleft, rright, Q,P1,P2 = image_rectify(left, right)
    cv2.imwrite("Rectified_left.png", rleft)
    cv2.imwrite("Rectified_right.png", rleft)
    print("Rectifying disparity done")

    disparity = compute_disparity(rleft, rright,P1,P2)
    depth_vis = disparity.copy()

    # Normalize to 0-255
    depth_vis = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
    depth_vis = depth_vis.astype(np.uint8)

    cv2.imwrite("depth_map_vis.png", depth_vis)
    print("Computed disparity done")

    points = disparity_to_points(
        disparity,
        Q
    )

    print("Generated points:", points.shape)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd = pcd.voxel_down_sample(0.2)

    print(pcd)

    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
        pcd,
        voxel_size=0.2
    )

    o3d.visualization.draw_geometries([voxel_grid])


if __name__ == "__main__":
    main()