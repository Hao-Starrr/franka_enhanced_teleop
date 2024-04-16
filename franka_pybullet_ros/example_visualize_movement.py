# 会启动一个panda机器人，并读取fep_state_to_pid-corrected-torque_55s_dataset.csv文件，然后按照这个文件中的动作进行动作控制。
# 没有完整的环境，背景是蓝色，但是可以看到机器人动作的变化。


import time
import pybullet as p

from panda_robot import FrankaPanda
from bullet_client import BulletClient
from movement_datasets import read_fep_dataset

INCLUDE_GRIPPER = True
DTYPE = 'float64'
SAMPLING_RATE = 1e-3  # 1000Hz sampling rate
FEP_MOVEMENT_DATASET_PATH = "./movement_datasets/fep_state_to_pid-corrected-torque_55s_dataset.csv"


def main():
    # Basic Setup of environment
    bc = BulletClient(p.GUI)
    bc.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    bc.configureDebugVisualizer(
        rgbBackground=[33. / 255., 90. / 255., 127. / 255.])
    bc.setTimeStep(SAMPLING_RATE)
    bc.setGravity(0, 0, -9.81)

    # Setup robot
    panda_robot = FrankaPanda(
        bc, include_gripper=INCLUDE_GRIPPER, simple_model=False)

    # Read FEP movement dataset, discarding everything except the joint positions for each sampling point as PyBullet
    # can be set to figure joint torques out by itself and only requires desired joint positions.
    pos, _, _, _ = read_fep_dataset(FEP_MOVEMENT_DATASET_PATH, DTYPE)

    # Set up variables for simulation loop
    dataset_length = pos.shape[0]
    period = 1 / SAMPLING_RATE
    counter_seconds = -1

    # start simulation loop
    for i in range(dataset_length):
        # Print status update every second of the simulation
        if i % period == 0:
            counter_seconds += 1
            print("Passed time in simulation: {:>4} sec".format(
                counter_seconds))

        # Select data of current position, then convert to list as PyBullet does not yet support numpy arrays as
        # parameters
        current_pos = pos[i].tolist()
        panda_robot.set_target_positions(current_pos)

        # Perform simulation step
        bc.stepSimulation()
        time.sleep(SAMPLING_RATE)

    # Exit Simulation
    bc.disconnect()
    print("Simulation end")


if __name__ == '__main__':
    main()
