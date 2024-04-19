# 只有仿真没有可视化，把
# /franka_physics_velocity_controller
# /gripper_command
# 的信号输入给bc
# 然后通过bc控制panda的动作
# /joint 是反馈回来的joint state


from env_ros import FrankaPandaEnvRosPhysics
import pybullet as p

# object_from_list -> frequency: 850 Hz

env = FrankaPandaEnvRosPhysics(connection_mode=p.DIRECT,
                               frequency=1000.,
                               controller='position',
                               include_gripper=True,
                               simple_model=True,
                               object_from_sdf=False,
                               object_from_list=True)

env.simulation_loop()
