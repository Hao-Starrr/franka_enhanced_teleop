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
from ikflow.utils import set_seed
from ikflow.model_loading import get_ik_solver


model = GraspSampler()

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

plane_id = p.loadURDF('plane.urdf', basePosition=[0., 0., -0.626], useFixedBase=True)
object_id = p.loadURDF('duck_vhacd.urdf', basePosition=[0.65, 0.0, 0.2], globalScaling=0.7)
# object_id = p.loadURDF('../006_mustard_bottle/tsdf/textured.urdf', basePosition=[0.6, 0.0, 0.0],
#                        baseOrientation=[0, 0, 0, 1], globalScaling=8, useFixedBase=False)
# filename = '../../ycb_dataset_mesh/ycb/014_lemon/google_16k/textured_convex.obj'
#
# import trimesh
# mesh = trimesh.load_mesh(filename)
#
# bounds = np.array(mesh.bounds)
# bounds = np.min(bounds[1] - bounds[0])
# if bounds > 0.04:
#     scale = 0.04 / bounds
# else:
#     scale = 1
#
# center_mass = np.array(mesh.center_mass) * scale
#
# collisionShapeId_VHACD = p.createCollisionShape(shapeType=p.GEOM_MESH,
#                                                 fileName=filename, meshScale=[scale, scale, scale])
# object_id = p.createMultiBody(baseMass=0.1,
#                               baseCollisionShapeIndex=collisionShapeId_VHACD,
#                               basePosition=[0.65, 0.0, 0.2],
#                               baseInertialFramePosition=center_mass)

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

device = "cpu"
dtype = torch.float32
chain = pk.build_serial_chain_from_urdf(open("KUKA_IIWA_URDF/iiwa7.urdf").read(), "iiwa_link_ee")
chain = chain.to(dtype=dtype, device=device)
q_orig = robot.get_joint_state()
flag = False

set_seed()
ik_solver, hyper_parameters = get_ik_solver("iiwa7_full_temp_nsc_tpm")

# draw_grasp_poses(H[:H.shape[0]//2], robot='kuka')

import time
start = time.time()
print(H.shape)
H = torch.tensor(H[np.arange(2000) % H.shape[0]], dtype=dtype, device=device)
ee = torch.cat([H[:, :3, 3], pk.matrix_to_quaternion(H[:, :3, :3])], dim=1)
mask = torch.zeros(ee.size(0), dtype=torch.bool, device=device)

with torch.inference_mode():
    q = ik_solver.solve_n_poses(ee, refine_solutions=False, return_detailed=False)

    for i in range(3):
        m = chain.forward_kinematics(q[~mask, :], end_only=True).get_matrix()
        pos = ee[~mask, :3] - m[:, :3, 3]
        rot = pk.matrix_to_axis_angle(H[~mask, :3, :3] @ m[:, :3, :3].transpose(1, 2))
        ee_e = torch.cat([pos, rot], dim=1)

        error = torch.linalg.norm(ee_e, dim=1) < 1e-3
        if error.all():
            mask[~mask] = error
            break

        J = chain.jacobian(q[~mask, :])
        q[~mask, :] += (J.transpose(1, 2) @ torch.linalg.solve(J @ J.transpose(1, 2), ee_e[:, :, None]))[:, :, 0]
        mask[~mask] = error

print(q.size(0), q[mask, :].size(0))

from gpis import RGPIS
joint_gpis = RGPIS(dim=7)
joint_gpis.train(q[mask, :].detach().numpy())

print(robot.get_joint_state())
print(time.time() - start)
while True:
    q_curr = robot.get_joint_state()
    grad = joint_gpis.get_surface_normal(q_curr[np.newaxis, :])[0]

    robot.control_arm_poses(q_curr + grad * 0.001)
    p.stepSimulation()

# for joints in q[mask, :]:
#     for i in range(len(joints)):
#         p.resetJointState(robot.robot_id, i, joints[i])
#     for i in range(10):
#         # p.stepSimulation()
#         time.sleep(0.01)
p.disconnect()
