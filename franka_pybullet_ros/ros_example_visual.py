# 只有可视化没有仿真
# 但能发布很多信息，比如摄像头图像

from env_ros import FrankaPandaEnvRosVisual
import pybullet as p

env = FrankaPandaEnvRosVisual(connection_mode=p.DIRECT,
                              frequency=30.,
                              include_gripper=True,
                              simple_model=False,
                              object_from_sdf=False,
                              object_from_list=True)

env.simulation_loop()
