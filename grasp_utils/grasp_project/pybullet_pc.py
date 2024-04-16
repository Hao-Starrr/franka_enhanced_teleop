import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import open3d as o3d

from pybullet_utils import get_point_cloud, draw_point_cloud, draw_grasp_poses, draw_grasp_frames
from grasp_sampler import GraspSampler
from riem_gmm import SE3GMM

model = GraspSampler()

p.connect(p.GUI)
p.configureDebugVisualizer(rgbBackground=[0, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
p.resetDebugVisualizerCamera(1, 0, 0, [0, 0, 0])
plane_id = p.loadURDF('plane.urdf', basePosition=[0., 0., -0.626], useFixedBase=True)
object_id = p.loadURDF('duck_vhacd.urdf', basePosition=[0.4, 0.0, 0.1], globalScaling=1.)
table_id = p.loadURDF('/table/table.urdf', basePosition=[0.5, 0., -0.626], globalScaling=1., useFixedBase=True)
# p.loadURDF('../024_bowl/tsdf/textured.urdf', basePosition=[0.25, 0., 0.25], globalScaling=10.)
p.setGravity(0, 0, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

width = 720
height = 640
fov = 60
aspect = width / height
near = 0.1
far = 5

view_matrix = p.computeViewMatrix([0, 0, 0.5], [0.5, 0, 0], [0, 0, 1])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

points = get_point_cloud(width, height, view_matrix, projection_matrix, object_id)

start = time.time()

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd = pcd.voxel_down_sample(voxel_size=0.01)
points = np.asarray(pcd.points)
points, H, w = model.sampler(points)


def transform_H(H):
    H_new = np.eye(4)[np.newaxis, :, :].repeat(H.shape[0], 0)
    H_new[:, :3, 0] = H[:, :3, 1]
    H_new[:, :3, 1] = -H[:, :3, 0]
    H_new[:, :3, 2] = H[:, :3, 2]
    H_new[:, :3, 3] = H[:, :3, 3] - (0.18 - 1.12169998e-01) * H[:, :3, 2]
    return H_new

# draw_grasp_poses(H[:6:5], color=[0.6, 0.6, 0.6])
H = transform_H(H)

gmm_reach = SE3GMM()
parameter = np.load('gmm_reachability.npz')
gmm_reach.mu = parameter['mu']
gmm_reach.sigma = parameter['sigma']
gmm_reach.pi = parameter['pi']
H_prob = gmm_reach.eval(H)

print(H.shape)
H = H[H_prob > 0.0005733336724437366]
draw_grasp_poses(H, color=[0.6, 0.6, 0.6], robot='kuka')

gmm_grasp = SE3GMM()
H = gmm_grasp.fit(H, n_clusters=4, n_iterations=10)
H = gmm_grasp.mu

print(time.time() - start)
import pytransform3d.trajectories

T = np.zeros(6)
T[3] = -1
# T[4] = 0.5
# T[0] = -np.pi/2
T[1] = np.pi / 2

for i in range(2000):
    grad = gmm_grasp.grad(T)
    norm = np.linalg.norm(grad)
    if norm == 0:
        norm = 1
    if i % 20 == 0:
        draw_grasp_poses(pytransform3d.trajectories.transforms_from_exponential_coordinates(T[np.newaxis, :]),
                         color=[0, 1, 0], robot='kuka')
    if np.linalg.norm(grad) < 0.6:
        break
    T[:3] += 1e-2 * grad[:3] / np.linalg.norm(grad[:3])
    T[3:] += 5e-3 * grad[3:] / np.linalg.norm(grad[3:])
print(i)


draw_grasp_poses(H, color=[1, 0, 0], robot='kuka')

object_pos, object_orn = p.getBasePositionAndOrientation(object_id)
object_t = np.array(object_pos)
object_R = np.array(p.getMatrixFromQuaternion(object_orn)).reshape((3, 3))
object_H = np.eye(4)
object_H[:3, :3] = object_R
object_H[:3, 3] = object_t
points_new = (object_R.T @ points.T - object_R @ object_t[:, np.newaxis]).T

draw_grasp_poses(np.linalg.inv(object_H) @ H, color=[1, 0, 0], robot='kuka')

ab = np.array([1, 1, 1])
print(ab[ab == 0].shape[0] == 0)

draw_point_cloud(points)
draw_point_cloud(points_new)
while True:
    p.stepSimulation()
    time.sleep(0.03)

p.disconnect()
