import numpy as np
import time
import pybullet as p
import pybullet_data as pd

p.connect(p.GUI)
p.configureDebugVisualizer(rgbBackground=[0, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
p.resetDebugVisualizerCamera(1, 0, 0, [0, 0, 0])
plane_id = p.loadURDF('plane.urdf', basePosition=[0., 0., -0.626], useFixedBase=True)
object_id = p.loadURDF('duck_vhacd.urdf', basePosition=[0.65, 0.0, 0.03], globalScaling=1.)
table_id = p.loadURDF('/table/table.urdf', basePosition=[0.5, 0., -0.626], globalScaling=1., useFixedBase=True)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

from robot import KUKASAKE

robot = KUKASAKE([0, 0, 0], [0, 0, 0, 1])
robot.reset_arm_poses()
robot.open_gripper()
# robot.close_gripper()
robot.control_arm_poses(robot.arm_rest_poses)
for i in range(1000):
    p.stepSimulation()

while True:
    # robot.delete_gripper_frame()
    # robot.delete_camera_frame()
    p.stepSimulation()
    robot.get_image()
    time.sleep(0.01)
    # robot.draw_gripper_frame()
    # robot.draw_camera_frame()

p.disconnect()