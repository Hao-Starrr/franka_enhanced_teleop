import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import open3d as o3d


def get_point_cloud(width, height, view_matrix, proj_matrix):
    image_arr = p.getCameraImage(width=width, height=height,
                                 viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                                 renderer=p.ER_TINY_RENDERER)
    depth = image_arr[3]
    segmentation = image_arr[4]

    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z, s = x.reshape(-1), y.reshape(-1), depth.reshape(-1), segmentation.reshape(-1)
    h = np.ones_like(z)

    pixels = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    pixels = pixels[z < 0.999]
    s = s[z < 0.999]

    pixels = pixels[s == 1]

    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3: 4]
    points = points[:, :3]

    return points


p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.loadURDF('plane.urdf')
p.loadURDF('duck_vhacd.urdf', basePosition=[0.0, 0.0, 0.1])
p.setGravity(0, 0, 0)
width = 480
height = 360

fov = 60
aspect = width / height
near = 0.1
far = 5

view_matrix = p.computeViewMatrix([0.1, 0, 0.5], [0, 0, 0], [0, 0, 1])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

points = get_point_cloud(width, height, view_matrix, projection_matrix)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd = pcd.voxel_down_sample(voxel_size=0.01)
points = np.asarray(pcd.points)

print(points.shape)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 0, 0, 1], radius=0.01)
collisionShapeId = -1
for point in points:
    mb = p.createMultiBody(baseMass=0,
                           baseCollisionShapeIndex=collisionShapeId,
                           baseVisualShapeIndex=visualShapeId,
                           basePosition=point,
                           useMaximalCoordinates=False)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
while True:
    p.stepSimulation()
    time.sleep(0.03)

p.disconnect()
