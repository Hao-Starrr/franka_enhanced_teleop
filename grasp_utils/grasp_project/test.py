import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import torch
import pytorch_kinematics as pk
import random

import pytransform3d.trajectories
import pytransform3d.trajectories as pytraj


random.seed(0)
q_temp = np.random.rand(7)

device = "cpu"
dtype = torch.float32
chain = pk.build_serial_chain_from_urdf(
    open("KUKA_IIWA_URDF/iiwa7.urdf").read(), "iiwa_link_ee")
chain = chain.to(dtype=dtype, device=device)
matrix1 = chain.forward_kinematics(q_temp, end_only=True).get_matrix()
T = pytransform3d.trajectories.exponential_coordinates_from_transforms(
    matrix1[0].numpy())
J1 = chain.jacobian(q_temp)

print(T)


kuka_model = rtb.models.iiwa7()
m = kuka_model.fkine(q_temp)  # return a SE3 object
matrix2 = m.A  # convert to 4x4 matrix
T_test = pytraj.exponential_coordinates_from_transforms([matrix2])[
    0]
J2 = kuka_model.jacob0(q_temp)
print(T_test)

print("diff")
print(np.linalg.norm(J1 - J2))
