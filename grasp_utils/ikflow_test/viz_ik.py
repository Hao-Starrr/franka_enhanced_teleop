import time
import math
import torch

import pybullet as p
import pytorch_kinematics as pk
from ikflow.utils import set_seed
from ikflow.model_loading import get_ik_solver

# get Jacobian in parallel and use CUDA if available
N = 1
device = "cpu"
dtype = torch.float32

p.connect(p.GUI)
robot = p.loadURDF("KUKA_IIWA_URDF/iiwa7.urdf")
chain = pk.build_serial_chain_from_urdf(open("KUKA_IIWA_URDF/iiwa7.urdf").read(), "iiwa_link_ee")

set_seed()
ik_solver, hyper_parameters = get_ik_solver("iiwa7_full_temp_nsc_tpm")

chain = chain.to(dtype=dtype, device=device)

batch = 1000
# q = torch.rand(batch, 7, dtype=dtype, device=device, requires_grad=True) * 0.1
ee_e = torch.tensor([0.4, 0., 0.4, math.pi, 0, 0], device=device)[None, :].repeat((batch, 1))
R = pk.axis_angle_to_matrix(ee_e[:, 3:])
quat = pk.axis_angle_to_quaternion(ee_e[:, 3:])
mask = torch.zeros(batch, dtype=torch.bool, device=device)

ee = torch.cat([ee_e[:, :3], quat], dim=1)
mask = torch.zeros(ee_e.size(0), dtype=torch.bool, device=device)

import time

start = time.time()

with torch.inference_mode():
    q = ik_solver.solve_n_poses(ee, refine_solutions=False, return_detailed=False)

    for i in range(3):
        m = chain.forward_kinematics(q[~mask, :], end_only=True).get_matrix()
        pos = ee[~mask, :3] - m[:, :3, 3]
        rot = pk.matrix_to_axis_angle(R[~mask] @ m[:, :3, :3].transpose(1, 2))
        ee_e = torch.cat([pos, rot], dim=1)

        error = torch.linalg.norm(ee_e, dim=1) < 1e-3
        if error.all():
            mask[~mask] = error
            break

        J = chain.jacobian(q[~mask, :])
        q[~mask, :] += (J.transpose(1, 2) @ torch.linalg.solve(J @ J.transpose(1, 2), ee_e[:, :, None]))[:, :, 0]
        mask[~mask] = error

print(time.time() - start)
print(q.size(0), q[mask, :].size(0))
for joints in q[mask, :]:
    for i in range(len(joints)):
        p.resetJointState(robot, i + 1, joints[i])
    for i in range(1):
        p.stepSimulation()
        time.sleep(0.01)

p.disconnect()
