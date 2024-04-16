import time
import math
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
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=60, cameraPitch=-15, cameraTargetPosition=[0., 0., 0.])
# plane_id = p.loadURDF('plane.urdf', basePosition=[0., 0., -0.626], useFixedBase=True)
object_id = p.loadURDF('duck_vhacd.urdf', basePosition=[0.65, 0.0, 0.1], baseOrientation=[0.7071068, 0, 0, 0.7071068], globalScaling=0.7)

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
# draw_point_cloud(points)

device = "cpu"
dtype = torch.float32
chain = pk.build_serial_chain_from_urdf(open("KUKA_IIWA_URDF/iiwa7.urdf").read(), "iiwa_link_ee")
chain = chain.to(dtype=dtype, device=device)
q_orig = robot.get_joint_state()
flag = False

ik_solver, hyper_parameters = get_ik_solver("iiwa7_full_temp_nsc_tpm")

from gpis import RGPIS

joint_gpis = RGPIS(dim=7)
train_flag = False
train_loop_flag = True
simulation_loop_flag = True


def gpis_loop():
    global joint_gpis
    global H
    global train_flag
    global train_loop_flag
    global ik_solver
    global chain

    start = time.time()
    # print('gpis start')
    joint_gpis_new = RGPIS(dim=7)

    H_ = torch.tensor(H[np.arange(100) % H.shape[0]], dtype=dtype, device=device)
    ee = torch.cat([H_[:, :3, 3], pk.matrix_to_quaternion(H_[:, :3, :3])], dim=1)
    mask = torch.zeros(ee.size(0), dtype=torch.bool, device=device)

    with torch.inference_mode():
        set_seed(0)
        q = ik_solver.solve_n_poses(ee, refine_solutions=False, return_detailed=False)
        for i in range(3):
            m = chain.forward_kinematics(q[~mask, :], end_only=True).get_matrix()
            pos = ee[~mask, :3] - m[:, :3, 3]
            rot = pk.matrix_to_axis_angle(H_[~mask, :3, :3] @ m[:, :3, :3].transpose(1, 2))
            ee_e = torch.cat([pos, rot], dim=1)

            error = torch.linalg.norm(ee_e, dim=1) < 1e-3
            if error.all():
                mask[~mask] = error
                break

            J = chain.jacobian(q[~mask, :])
            q[~mask, :] += (J.transpose(1, 2) @ torch.linalg.solve(J @ J.transpose(1, 2), ee_e[:, :, None]))[:, :, 0]
            mask[~mask] = error

    q = q[mask, :].detach().numpy()
    if q.shape[0] > 0:
        joint_gpis_new.train(q)
        joint_gpis = joint_gpis_new
    train_flag = True
    train_loop_flag = True
    print('gpis end', 1/(time.time() - start))


def circular_path(index):
    return [0.45 * math.cos(index * 0.005), 0.45 * math.sin(index * 0.005), 0.2]
def linear_path(index):
    return [0.45, 0.45 * math.sin(index * 0.005), 0.1 * math.sin(index * 0.005) + 0.2]
def sinusoidal_path(index):
    return [0.1 * math.cos(index * 0.05) + 0.45, 0.35 * math.sin(index * 0.005), 0.2]


import threading
gpis_loop()

pos, orn = p.getBasePositionAndOrientation(object_id)
R = p.getMatrixFromQuaternion(orn)
pos = np.array(pos)
R = np.array(R).reshape((3, 3))
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = pos

trajectory = sinusoidal_path

for i in range(int(math.pi / 0.005)):
    print(i, int(math.pi / 0.005))
    p.addUserDebugLine(trajectory(i), trajectory(i+1), lineColorRGB=[1, 0, 0])
    p.addUserDebugLine(trajectory(-i), trajectory(-i - 1), lineColorRGB=[1, 0, 0])


index = 0
while True:
    if train_loop_flag:
        train_loop_flag = False
        p1 = threading.Thread(target=gpis_loop)
        p1.start()

    pos_new = trajectory(index)
    # orn_new = [0.7071068, 0, 0, 0.7071068]
    orn_new = p.getQuaternionFromEuler([0.005 * index, 0, 0])
    p.resetBasePositionAndOrientation(object_id, pos_new, orn_new)
    R_new = p.getMatrixFromQuaternion(orn_new)
    pos_new = np.array(pos_new)
    R_new = np.array(R_new).reshape((3, 3))
    T_new = np.eye(4)
    T_new[:3, :3] = R_new
    T_new[:3, 3] = pos_new

    H = T_new @ np.linalg.inv(T) @ H

    q_curr = robot.get_joint_state()
    grad = joint_gpis.get_surface_normal(q_curr[np.newaxis, :])[0]
    mean = joint_gpis.get_distance(q_curr[np.newaxis, :])[0, 0]

    robot.control_arm_poses(q_curr + grad * min(.5, mean))

    p.stepSimulation()
    time.sleep(1/240.)
    T = T_new
    index += 1

p.disconnect()
