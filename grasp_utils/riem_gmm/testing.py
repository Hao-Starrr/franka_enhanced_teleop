import numpy as np
import time
import pybullet as p

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.5])

p.setRealTimeSimulation(0)
p.setGravity(0, 0, 0)

robot = p.loadURDF("robots/iiwa7_base.urdf", basePosition=[0.25, 0, 0], useFixedBase=True,
                   flags=p.URDF_USE_SELF_COLLISION)
p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)

arm_joint_ids = [0, 1, 2, 3, 4, 5, 6]
gripper_joint_ids = [21, 24]

pi = np.pi
q_min = np.array([-pi*(170/180), -pi*(120/180), -pi*(170/180),
                  -pi*(120/180), -pi*(170/180), -pi*(120/180),
                  -pi*(175/180), -1.57075, -1.57075])
q_max = np.array([ pi*(170/180),  pi*(120/180),  pi*(170/180),
                   pi*(120/180),  pi*(170/180),  pi*(120/180),
                   pi*(175/180), 0.27, 0.27])

n_joint = p.getNumJoints(robot)
for i in range(n_joint):
    print(i, p.getJointInfo(robot, i))

# for gj in gripper_joint_ids:
#     p.resetJointState(robot, gj, -pi/2)
print(p.getLinkState(robot, 7)[4])
print(p.getMatrixFromQuaternion(p.getLinkState(robot, 7)[5]))
import pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())
while True:
    p.stepSimulation()
    time.sleep(0.03)
