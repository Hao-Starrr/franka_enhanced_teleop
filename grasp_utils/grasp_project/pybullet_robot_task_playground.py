import trimesh
import copy

import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import open3d as o3d

from pybullet_utils import get_point_cloud, draw_point_cloud, draw_grasp_poses, draw_grasp_frames
from grasp_sampler import GraspSampler
from riem_gmm import SE3GMM
from robot import KUKASAKE

import torch
import pytorch_kinematics as pk
import pytransform3d.trajectories
from pytransform3d.trajectories import exponential_coordinates_from_transforms
from pytransform3d.trajectories import transforms_from_exponential_coordinates
from scipy.spatial.transform import Rotation as R

import roboticstoolbox as rtb
from spatialmath import SE3


p.connect(p.GUI)
dt = 1. / 240.
SOLVER_STEPS = 1000  # a bit more than default helps in contact-rich tasks
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

plane_id = p.loadURDF('plane.urdf', basePosition=[
                      0., 0., -0.626], useFixedBase=True)
# object_id = p.loadURDF('duck_vhacd.urdf', basePosition=[0.65, 0.0, 0.3], globalScaling=0.7)
# object_id = p.loadURDF('../006_mustard_bottle/tsdf/textured.urdf', basePosition=[0.6, 0.0, 0.0],
#                        baseOrientation=[0, 0, 0, 1], globalScaling=8, useFixedBase=False)
filename = '../../ycb_dataset_mesh/ycb/014_lemon/google_16k/textured.obj'

mesh = trimesh.load_mesh(filename)

bounds = np.array(mesh.bounds)
bounds = np.min(bounds[1] - bounds[0])
if bounds > 0.04:
    scale = 0.04 / bounds
else:
    scale = 1

center_mass = np.array(mesh.center_mass) * scale

collisionShapeId_VHACD = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                                fileName=filename, meshScale=[scale, scale, scale])
object_id = p.createMultiBody(baseMass=0.1,
                              baseCollisionShapeIndex=collisionShapeId_VHACD,
                              basePosition=[0.65, 0.0, 0.1],
                              baseInertialFramePosition=center_mass)

# table_id = p.loadURDF('/table/table.urdf', basePosition=[0.5, 0., -0.626], globalScaling=1., useFixedBase=True)
# p.setGravity(0, 0, -9.81)
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


############################# all above is just setup pybullet #######################

# sample
model = GraspSampler()

points = get_point_cloud(640, 480, robot.get_view_matrix(),
                         robot.projectionMatrix, object_id)
print("points shape: ", points.shape)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd = pcd.voxel_down_sample(voxel_size=0.01)
points = np.asarray(pcd.points)
points, H, w = model.sampler(points)
# H is grasp pose, w is grasp width


############################# choose the grasp pose ##################################
# reachability
gmm_reach = SE3GMM()
parameter = np.load('gmm_reachability.npz')
gmm_reach.mu = parameter['mu']
gmm_reach.sigma = parameter['sigma']
gmm_reach.pi = parameter['pi']
print("H shape: ", H.shape)


# why??????????????????
H_prob = gmm_reach.eval(H)
H = H[H_prob > 0.0005733336724437366]

# grasp direction filter
H = H[H[:, 2, 3] > 0.05]

# draw_grasp_poses(H, color=[0.6, 0.6, 0.6], robot='kuka')
draw_point_cloud(points)

##################### fit the distribution, then we can take the gradient ##################
gmm_grasp = SE3GMM()
H_sample = gmm_grasp.fit(H, n_clusters=min(8, H.shape[0]), n_iterations=10)
# H_mu = gmm_grasp.mu
# prob = gmm_grasp.eval(H)

draw_grasp_poses(H, color=[1, 0., 0.], robot='kuka')


T = pytransform3d.trajectories.exponential_coordinates_from_transforms(
    robot.get_ee_transform())
for i in range(100):
    grad = gmm_grasp.grad(T)
    norm = np.linalg.norm(grad)
    if norm == 0:
        norm = 1
    if i % 5 == 0:
        draw_grasp_poses(pytransform3d.trajectories.transforms_from_exponential_coordinates(T[np.newaxis, :]),
                         color=[0, 1, 0], robot='kuka')
    if np.linalg.norm(grad) < 0.6:
        break
    print(np.linalg.norm(grad))
    if np.linalg.norm(grad) > 1e2:
        grad = grad / np.linalg.norm(grad) * 1e2
    T += 1e-4 * grad
    # T[:3] += 1e-3 * grad[:3] / np.linalg.norm(grad[:3])
    # T[3:] += 1e-3 * grad[3:] / np.linalg.norm(grad[3:])


kuka_model = rtb.models.iiwa7()
# 增加维度从7变成1x7
q_temp = robot.get_joint_state()
print("q_temp shape: ", q_temp.shape)

while True:
    # 算出ee pose
    m = kuka_model.fkine(q_temp).A

    # 换成指数坐标
    T = exponential_coordinates_from_transforms(m)

    # 求梯度(指数坐标里的速度)
    grad = gmm_grasp.grad(T)
    if np.linalg.norm(grad) > 1e1:
        grad = grad / np.linalg.norm(grad) * 1e1

    # 梯度也在指数坐标里,所以要在这个坐标里加上速度
    T_new = copy.deepcopy(T)
    T_new[0:3] = T[0:3] + 1e-3 * grad[0:3]
    T_new[3:6] = T[3:6] + 1e-3 * grad[3:6]
    # 转换回矩阵
    T_new = transforms_from_exponential_coordinates(T_new)

    # 计算位置差
    pos = T_new[0:3, 3] - m[0:3, 3]
    # 计算姿态差
    rot = T_new[:3, :3] @ m[:3, :3].T
    rot_vector = R.from_matrix(rot).as_rotvec()

    ee_error = np.hstack((pos, rot_vector))

    # 此时的J
    J = kuka_model.jacob0(q_temp)

    # q velocity
    q_dot = np.linalg.pinv(J) @ ee_error.reshape(-1, 1)  # 使用伪逆计算关节速度
    q_temp += q_dot.flatten()  # * 0.01

    q_norm = np.linalg.norm(q_dot)
    if q_norm < 0.005:
        break

    robot.control_arm_poses(q_temp)
    robot.reset_arm_poses(q_temp)
    # p.stepSimulation()
    # time.sleep(0.01)


robot.reset_arm_poses(q_temp[0])
robot.control_arm_poses(q_temp[0])

for i in range(240 * 2):
    robot.control_arm_poses(q_temp[0])
    robot.close_gripper()
    p.stepSimulation()


p.setGravity(0, 0, -9.81)

for i in range(240 * 10):
    q_curr = robot.get_joint_state()
    robot.control_arm_poses(0.01 * (q_orig-q_curr) + q_curr)
    p.stepSimulation()

# p.disconnect()
