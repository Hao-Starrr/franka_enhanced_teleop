import time
import math
import torch

import pybullet as p
import pytorch_kinematics as pk

p.connect(p.GUI)
robot = p.loadURDF("KUKA_IIWA_URDF/iiwa7.urdf")
chain = pk.build_serial_chain_from_urdf(open("KUKA_IIWA_URDF/iiwa7.urdf").read(), "iiwa_link_ee")

# get Jacobian in parallel and use CUDA if available
N = 1
d = "cpu"
dtype = torch.float32

chain = chain.to(dtype=dtype, device=d)

batch = 1000
q = torch.rand(batch, 7, dtype=dtype, device=d, requires_grad=True) * 0.1
ee = torch.tensor([0.4, 0., 0.4, math.pi, 0, 0])[None, :].repeat((batch, 1))
R = pk.axis_angle_to_matrix(ee[:, 3:])
mask = torch.zeros(batch, dtype=torch.bool)

joint_upper_limit = torch.tensor([math.radians(170), math.radians(120), math.radians(170),
                                  math.radians(120), math.radians(170), math.radians(120),
                                  math.radians(175)])


@torch.jit.script
def inverse_jacobian(q, ee_e, J, joint_upper_limit):
    JJ = J @ J.transpose(1, 2)
    JJ[:, [0, 1, 2, 3, 4, 5], [0, 1, 2, 3, 4, 5]] += 1e-3
    J_inv = J.transpose(1, 2) @ torch.linalg.inv(JJ)
    q += (J_inv @ ee_e[:, :, None])[:, :, 0]
    # pm = -J_inv @ J
    # pm[:, [0, 1, 2, 3, 4, 5, 6], [0, 1, 2, 3, 4, 5, 6]] += 1
    # k = 1e-3 / 7
    # q += (J_inv @ ee_e[:, :, None] + pm @ (k * q / (2 * joint_upper_limit[None, :]))[:, :, None])[:, :, 0]
    return q


start = time.time()

with torch.no_grad():
    for i in range(40):
        m = chain.forward_kinematics(q[~mask, :], end_only=True).get_matrix()
        pos = ee[~mask, :3] - m[:, :3, 3]
        rot = pk.matrix_to_axis_angle(R[~mask] @ m[:, :3, :3].transpose(1, 2))
        ee_e = torch.cat([pos, rot], dim=1)

        error = torch.linalg.norm(ee_e, dim=1) < 1e-5
        if error.all():
            mask[~mask] = error
            break

        J = chain.jacobian(q[~mask, :])
        q[~mask, :] = inverse_jacobian(q[~mask, :], ee_e, J, joint_upper_limit)
        mask[~mask] = error

print(time.time() - start)
print(mask.sum())

for joints in q[mask, :]:
    for i in range(len(joints)):
        p.resetJointState(robot, i + 1, joints[i])
    for i in range(1):
        p.stepSimulation()
        time.sleep(0.01)

p.disconnect()
