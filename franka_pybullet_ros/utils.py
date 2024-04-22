import numpy as np
import pybullet as p


def draw_point_cloud(points):
    colors = np.zeros(points.shape)
    colors[:, 0] = 1 - ((points[:, 2] - points[:, 2].min()) /
                        (points[:, 2].max() - points[:, 2].min()) * 1)
    colors[:, 1] = ((points[:, 2] - points[:, 2].min()) /
                    (points[:, 2].max() - points[:, 2].min()) * 1)
    colors[:, 2] = 1
    p.addUserDebugPoints(points, colors, pointSize=2.)


def draw_grasp_poses(H, color=[1, 0, 0], robot='franka'):
    if robot == 'franka':
        gripper_vertices = np.array([[4.10000000e-02, 0, 6.59999996e-02, 1],
                                     [4.10000000e-02, 0, 1.12169998e-01, 1],
                                     [-4.100000e-02, 0, 6.59999996e-02, 1],
                                     [-4.100000e-02, 0, 1.12169998e-01, 1],
                                     [0, 0, 0, 1],
                                     [0, 0, 6.59999996e-02, 1]])
        gripper_edges = np.array([[4, 5], [0, 2], [0, 1], [2, 3]])
    else:
        gripper_vertices = np.array([[0, 4.10000000e-02, 0.105, 1],
                                     [0, 4.10000000e-02, 0.18, 1],
                                     [0, -4.10000000e-02, 0.105, 1],
                                     [0, -4.10000000e-02, 0.18, 1],
                                     [0, 0, 0, 1],
                                     [0, 0, 0.105, 1]])
        gripper_edges = np.array([[4, 5], [0, 2], [0, 1], [2, 3]])
    for i in range(H.shape[0]):
        gripper_points = (H[i] @ gripper_vertices.T).T[:, :3]
        for u, v in gripper_edges:
            p.addUserDebugLine(
                gripper_points[u], gripper_points[v], lineColorRGB=color, lineWidth=2)


def draw_grasp_frames(H, scale=0.1):
    for i in range(H.shape[0]):
        p.addUserDebugLine(lineFromXYZ=H[i, :3, 3],
                           lineToXYZ=H[i, :3, 3] + H[i, :3, 0] * scale,
                           lineColorRGB=[1, 0, 0])
        p.addUserDebugLine(lineFromXYZ=H[i, :3, 3],
                           lineToXYZ=H[i, :3, 3] + H[i, :3, 1] * scale,
                           lineColorRGB=[0, 1, 0])
        p.addUserDebugLine(lineFromXYZ=H[i, :3, 3],
                           lineToXYZ=H[i, :3, 3] + H[i, :3, 2] * scale,
                           lineColorRGB=[0, 0, 1])
