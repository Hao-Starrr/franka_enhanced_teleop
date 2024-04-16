import time

import numpy as np
import open3d as o3d
import pybullet as p
import pybullet_data as pd
import pytorch_kinematics as pk
import torch

from grasp_sampler import GraspSampler
from ikflow.model_loading import get_ik_solver
from ikflow.utils import set_seed
from pybullet_utils import get_point_cloud, draw_point_cloud, draw_grasp_poses
from riem_gmm import SE3GMM
from robot import KUKASAKE

model = GraspSampler()

p.connect(p.GUI)
dt = 1. / 60.
SOLVER_STEPS = 100  # a bit more than default helps in contact-rich tasks
TIME_SLEEP = dt * 3  # for visualization
LATERAL_FRICTION = 1.0000
SPINNING_FRICTION = 0.0001
ROLLING_FRICTION = 0.0001
p.setPhysicsEngineParameter(fixedTimeStep=dt, numSolverIterations=SOLVER_STEPS,
                            useSplitImpulse=True, enableConeFriction=True,
                            splitImpulsePenetrationThreshold=0.0)
p.resetSimulation()

p.configureDebugVisualizer(rgbBackground=[0, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
p.resetDebugVisualizerCamera(1, 0, 0, [0, 0, 0])

plane_id = p.loadURDF('plane.urdf', basePosition=[0., 0., -0.626], useFixedBase=True)
object_id = p.loadURDF('duck_vhacd.urdf', basePosition=[0.65, 0.0, 0.2], globalScaling=0.7)

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

robot = KUKASAKE([0, 0, 0], [0, 0, 0, 1])
robot.reset_arm_poses(np.deg2rad([0, 30, 0, -60, 0, 90, 0]))
robot.open_gripper()
robot.control_arm_poses(np.deg2rad([0, 30, 0, -60, 0, 90, 0]))

p.changeDynamics(object_id, -1, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION,
                 rollingFriction=ROLLING_FRICTION)
for i in range(p.getNumJoints(robot.robot_id)):
    p.changeDynamics(robot.robot_id, i, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION,
                     rollingFriction=ROLLING_FRICTION)

for i in range(1000):
    p.stepSimulation()
    # time.sleep(0.01)

points = get_point_cloud(640, 480, robot.get_view_matrix(), robot.projectionMatrix, object_id)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd = pcd.voxel_down_sample(voxel_size=0.01)
points = np.asarray(pcd.points)
points, H, w = model.sampler(points)

gmm_reach = SE3GMM()
parameter = np.load('gmm_reachability.npz')
gmm_reach.mu = parameter['mu']
gmm_reach.sigma = parameter['sigma']
gmm_reach.pi = parameter['pi']
H_prob = gmm_reach.eval(H)

H = H[H_prob > 0.0005733336724437366]
H = H[H[:, 2, 3] > 0.05]

# draw_grasp_poses(H, color=[0.6, 0.6, 0.6], robot='kuka')
draw_point_cloud(points)

pos, orn = p.getBasePositionAndOrientation(object_id)
R = p.getMatrixFromQuaternion(orn)
pos = np.array(pos)
R = np.array(R).reshape((3, 3))
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = pos

pos_new = [0.65, 0.5, 0.2]
orn_new = [1, 0, 0, 0]
p.resetBasePositionAndOrientation(object_id, pos_new, orn_new)
# R_new = p.getMatrixFromQuaternion(orn_new)
# pos_new = np.array(pos_new)
# R_new = np.array(R_new).reshape((3, 3))
# T_new = np.eye(4)
# T_new[:3, :3] = R_new
# T_new[:3, 3] = pos_new
#
# draw_grasp_poses(T_new @ np.linalg.inv(T) @ H, color=[0.6, 0.6, 0.6], robot='kuka')


print(pos, orn, points.shape)

index = 0
import math
while True:
    pos_new = [0.65, 0.25 * math.sin(index * 0.01), 0.2]
    orn_new = [0, 0, 0, 1]
    p.resetBasePositionAndOrientation(object_id, pos_new, orn_new)

    p.stepSimulation()
    time.sleep(1/240.)
    index += 1

p.disconnect()
