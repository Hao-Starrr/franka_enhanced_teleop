# 会启动三个进程，分别运行物理仿真和可视化。
# 物理仿真进程运行run_physics函数，可视化进程运行run_visual函数。
# 两个进程共享数据，包括关节输入、夹爪状态、关节数据、TF、RGB图像、深度图像等。
# 会显示RGB图像和深度图像，并等待用户输入。还会显示可视化环境和可以交互的环境。
# 按q键退出程序。

import multiprocessing as mp
import time
import cv2
import numpy as np
from env_mp import run_physics, run_visual

if __name__ == "__main__":
    mp.set_start_method('spawn')  # Use 'spawn' method on Windows

    num_processes = 3
    simulation_time_steps = [0.01, 0.01, 0.01]
    total_simulation_steps = 100

    include_gripper = True
    object_from_sdf = True
    object_from_list = False

    joint_input = mp.Array('f', 7)
    gripper_input = mp.Value('i', 1)
    joint_data = mp.Array('f', 3 * 9)
    tf = mp.Manager().dict()
    image_rgb = mp.Array('i', 480 * 640 * 3)
    image_depth = mp.Array('f', 480 * 640)

    process_physics = mp.Process(target=run_physics, args=(
        joint_input, gripper_input, joint_data, tf, include_gripper, object_from_sdf, object_from_list))
    process_visual = mp.Process(target=run_visual, args=(
        joint_data, tf, image_rgb, image_depth, include_gripper, object_from_sdf, object_from_list))

    process_physics.start()
    process_visual.start()

    for i in range(200):
        with joint_input.get_lock():
            shared_np_array = np.frombuffer(
                joint_input.get_obj(), dtype=np.float32)
            shared_np_array[:] = np.sin([i / 100.] * 7)

        with gripper_input.get_lock():
            if i % 20 == 0:
                if gripper_input.value == 0:
                    gripper_input.value = 1
                else:
                    gripper_input.value = 0

        with image_rgb.get_lock():
            image_rgb_np_array = np.frombuffer(
                image_rgb.get_obj(), dtype=np.int32).reshape((480, 640, 3))
        with image_depth.get_lock():
            image_depth_np_array = np.frombuffer(
                image_depth.get_obj(), dtype=np.float32).reshape((480, 640))

        color_image = cv2.cvtColor(
            np.copy(image_rgb_np_array).astype(np.uint8), cv2.COLOR_RGB2BGR)
        depth_image = np.copy(image_depth_np_array)
        cv2.imshow('Color', color_image)
        cv2.imshow('Depth', depth_image)
        key = 0xFF & cv2.waitKey(1)
        if key == ord('q'):
            break
        time.sleep(0.1)

    cv2.destroyAllWindows()
    process_physics.join()
    process_visual.join()
