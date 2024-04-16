import copy
import os
import sys
from contextlib import contextmanager
import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import open3d as o3d
import trimesh
import tqdm

from pybullet_utils import get_point_cloud, draw_point_cloud, draw_grasp_poses, draw_grasp_frames
from grasp_sampler import GraspSampler
from riem_gmm import SE3GMM
from robot import KUKASAKE

import torch
import pytorch_kinematics as pk
import pytransform3d.trajectories

model = GraspSampler()

p.connect(p.DIRECT)
dt = 1. / 240.
SOLVER_STEPS = 500  # a bit more than default helps in contact-rich tasks
LATERAL_FRICTION = 100.0
SPINNING_FRICTION = 0.0001
ROLLING_FRICTION = 0.0001
p.setPhysicsEngineParameter(fixedTimeStep=dt, numSolverIterations=SOLVER_STEPS,
                            useSplitImpulse=True, enableConeFriction=False)

p.configureDebugVisualizer(rgbBackground=[0, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
p.resetDebugVisualizerCamera(1, 0, 0, [0, 0, 0])

device = "cpu"
dtype = torch.float32
chain = pk.build_serial_chain_from_urdf(open("KUKA_IIWA_URDF/iiwa7.urdf").read(), "iiwa_link_ee")
chain = chain.to(dtype=dtype, device=device)

PATH = '../../ycb_dataset_mesh/ycb/'

fail = np.zeros((5, len(os.listdir(PATH))))
success = np.zeros((5, len(os.listdir(PATH))))
not_feasible = np.zeros((5, len(os.listdir(PATH))))


@contextmanager
def suppress_stdout():
    fd = sys.stdout.fileno()

    def _redirect_stdout(to):
        sys.stdout.close()  # + implicit flush()
        os.dup2(to.fileno(), fd)  # fd writes to 'to' file
        sys.stdout = os.fdopen(fd, "w")  # Python writes to fd

    with os.fdopen(os.dup(fd), "w") as old_stdout:
        with open(os.devnull, "w") as file:
            _redirect_stdout(to=file)
        try:
            yield  # allow code to be run with the redirected stdout
        finally:
            _redirect_stdout(to=old_stdout)  # restore stdout.
            # buffering and flags such as
            # CLOEXEC may be different


bar1 = tqdm.tqdm(range(5), position=0, desc="Iterations", leave=True, colour='green', ncols=160)
for iteration_index in bar1:
    bar2 = tqdm.tqdm(os.listdir(PATH), position=1, desc="Objects", leave=False, colour='red', ncols=160)
    for file_index, file in enumerate(bar2):
        p.resetSimulation()
        p.setGravity(0, 0, 0)
        filename = PATH + file + '/google_16k/textured_convex.obj'

        mesh = trimesh.load_mesh(filename)

        bounds = np.array(mesh.bounds)
        bounds = np.min(bounds[1] - bounds[0])
        # if bounds > 0.03:
        #     scale = 0.03 / bounds
        # else:
        #     scale = 1
        scale = 1

        center_mass = np.array(mesh.center_mass) * scale

        collisionShapeId_VHACD = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                                        fileName=filename, meshScale=[scale, scale, scale])

        x_err = np.random.uniform(-0.05, 0.05)
        y_err = np.random.uniform(-0.05, 0.05)
        z_err = np.random.uniform(-0.05, 0.05)
        basepose = np.array([0.4 + x_err, 0.0 + y_err, 0.6 + z_err])
        object_id = p.createMultiBody(baseMass=0.1,
                                      baseCollisionShapeIndex=collisionShapeId_VHACD,
                                      basePosition=basepose,
                                      baseOrientation=[0, 0, 0, 1],
                                      baseInertialFramePosition=center_mass)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        with suppress_stdout():
            robot = KUKASAKE([0, 0, 0], [0, 0, 0, 1])
        robot.reset_arm_poses(robot.arm_rest_poses)
        robot.open_gripper()

        p.changeDynamics(object_id, -1, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION,
                         rollingFriction=ROLLING_FRICTION)
        for i in range(p.getNumJoints(robot.robot_id)):
            p.changeDynamics(robot.robot_id, i, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION,
                             rollingFriction=ROLLING_FRICTION)

        for i in range(1000):
            p.stepSimulation()

        points = get_point_cloud(640, 480, robot.get_view_matrix(), robot.projectionMatrix, object_id)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size=0.01)
        points = np.asarray(pcd.points)
        if points.shape[0] == 0:
            not_feasible[iteration_index, file_index] += 1
            bar2.set_description("Success: %3d, Failure: %3d, Not feasible: %3d)" % (
                np.sum(success), np.sum(fail), np.sum(not_feasible)))
            continue
        points, H, w = model.sampler(points)

        gmm_reach = SE3GMM()
        parameter = np.load('gmm_reachability.npz')
        gmm_reach.mu = parameter['mu']
        gmm_reach.sigma = parameter['sigma']
        gmm_reach.pi = parameter['pi']
        H_prob = gmm_reach.eval(H)

        H = H[H_prob > 0.0005733336724437366]
        # H = H[H[:, 2, 3] > 0.05]

        if H.shape[0] == 0:
            not_feasible[iteration_index, file_index] += 1
            bar2.set_description("Success: %3d, Failure: %3d, Not feasible: %3d)" % (
                np.sum(success), np.sum(fail), np.sum(not_feasible)))
            continue
        q_orig = robot.get_joint_state()
        gmm_grasp = SE3GMM()
        H_sample = gmm_grasp.fit(H, n_clusters=min(4, H.shape[0]), n_iterations=10)
        H_mu = gmm_grasp.mu
        draw_grasp_poses(H_mu, color=[1, 0., 0.], robot='kuka')
        # draw_point_cloud(points)

        q_temp = torch.tensor(robot.get_joint_state(), dtype=dtype, device=device, requires_grad=False)[None, :]
        max_iter = 5000
        for i in range(max_iter):
            m = chain.forward_kinematics(q_temp, end_only=True).get_matrix()
            T = pytransform3d.trajectories.exponential_coordinates_from_transforms(m[0].numpy())

            grad = gmm_grasp.grad(T)
            if np.linalg.norm(grad) > 5e0:
                grad = grad / np.linalg.norm(grad) * 5e0
            T_new = copy.deepcopy(T)
            T_new[:3] = T[:3] + 1e-3 * grad[:3]
            T_new[3:] = T[3:] + 1e-3 * grad[3:]
            T_new = torch.tensor(pytransform3d.trajectories.transforms_from_exponential_coordinates(T_new),
                                 dtype=dtype)[None, :, :]

            pos = T_new[:, :3, 3] - m[:, :3, 3]
            rot = pk.matrix_to_axis_angle(T_new[:, :3, :3] @ m[:, :3, :3].transpose(1, 2))
            ee_e = torch.cat([pos, rot], dim=1)

            J = chain.jacobian(q_temp)
            q_temp += (J.transpose(1, 2) @ torch.linalg.solve(J @ J.transpose(1, 2),
                                                              ee_e[:, :, None]))[:, :, 0]

            if torch.linalg.norm(
                    (J.transpose(1, 2) @ torch.linalg.solve(J @ J.transpose(1, 2), ee_e[:, :, None]))[:, :,
                    0]) < 0.0001:
                break
        robot.reset_arm_poses(q_temp[0])
        if i == max_iter - 1:
            not_feasible[iteration_index, file_index] += 1
            bar2.set_description("Success: %3d, Failure: %3d, Not feasible: %3d)" % (
                np.sum(success), np.sum(fail), np.sum(not_feasible)))
            continue

        p.resetBasePositionAndOrientation(object_id, basepose + center_mass, [0, 0, 0, 1])
        p.changeDynamics(object_id, -1, mass=0)
        position = 1.57075
        for i in range(240 * 2):
            position -= 1.57075 / (240 * 1)
            robot.control_gripper(position)
            time.sleep(1. / 240.)
        p.changeDynamics(object_id, -1, mass=0.1)
        p.setGravity(0, 0, -9.81)
        for i in range(240 * 2):
            q_curr = robot.get_joint_state()
            robot.control_arm_poses(0.01 * (q_orig - q_curr) + q_curr)
            p.stepSimulation()

        object_pos, _ = p.getBasePositionAndOrientation(object_id)
        if object_pos[2] > 0:
            success[iteration_index, file_index] += 1
        else:
            fail[iteration_index, file_index] += 1
        bar2.set_description("Success: %3d, Failure: %3d, Not feasible: %3d)" % (
            np.sum(success), np.sum(fail), np.sum(not_feasible)))

print(success)
print(fail)
print(not_feasible)
np.savez('results.npz', success=success, fail=fail, not_feasible=not_feasible)
p.disconnect()
