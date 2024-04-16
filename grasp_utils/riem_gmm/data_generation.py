import time
import numpy as np
import pybullet as p
from math import pi
from tqdm.contrib import itertools

p.connect(p.DIRECT)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.5])

p.resetSimulation()
p.setGravity(0, 0, 0)

p.setAdditionalSearchPath("./models")
robot = p.loadURDF("KUKA_IIWA_URDF/iiwa7.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)
dof = p.getNumJoints(robot)  # Virtual fixed joint between the flange and last link
joints = [0, 1, 2, 3, 4, 5, 6]

q_min = np.array(
    [-pi * (170 / 180), -pi * (120 / 180), -pi * (170 / 180), -pi * (120 / 180), -pi * (170 / 180), -pi * (120 / 180),
     -pi * (175 / 180)])
q_max = np.array(
    [pi * (170 / 180), pi * (120 / 180), pi * (170 / 180), pi * (120 / 180), pi * (170 / 180), pi * (120 / 180),
     pi * (175 / 180)])

q_ranges = []
for i in joints:
    q_ranges.append(np.linspace(q_min[i], q_max[i], 10))

Rs = []
ts = []
cols = []
file_index = 1

for q in itertools.product(*q_ranges):
    for q_id in range(len(q)):
        p.resetJointState(robot, q_id, q[q_id])

    link_data = p.getLinkState(robot, 7)
    t, R = link_data[4], p.getMatrixFromQuaternion(link_data[5])

    min_dist = 5
    for link1 in range(0, 6):
        for link2 in range((link1 + 2), 8):
            min_dist = min(min_dist, p.getClosestPoints(robot, robot, 5, link1, link2)[0][8])
    collistion_check = int(min_dist < 0)

    Rs.append(R)
    ts.append(t)
    cols.append(collistion_check)
    if len(Rs) >= 100000:
        np.savez("data/ee_data" + str(file_index * 100000) + ".npz", Rs=np.array(Rs), ts=np.array(ts), cols=np.array(cols))
        file_index += 1
        Rs = []
        ts = []
        cols = []


if len(Rs) > 0:
    np.savez("data/ee_data" + str((file_index - 1) * 100000 + len(Rs)) + ".npz", Rs=np.array(Rs), ts=np.array(ts), cols=np.array(cols))
    file_index += 1
    Rs = []
    ts = []
    cols = []