import os
import sys
import time
import numpy as np
import pybullet as p
from env import FrankaPandaEnv


class HideOutput(object):
    '''
    A context manager that block stdout for its scope, usage:

    with HideOutput():
        os.system('ls -l')
    '''

    def __init__(self, *args, **kw):
        sys.stdout.flush()
        self._origstdout = sys.stdout
        self._oldstdout_fno = os.dup(sys.stdout.fileno())
        self._devnull = os.open(os.devnull, os.O_WRONLY)

    def __enter__(self):
        self._newstdout = os.dup(1)
        os.dup2(self._devnull, 1)
        os.close(self._devnull)
        sys.stdout = os.fdopen(self._newstdout, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._origstdout
        sys.stdout.flush()
        os.dup2(self._oldstdout_fno, 1)


class Rate(object):
    def __init__(self, hz):
        self.time_function = time.perf_counter_ns
        self.last_time = self.time_function()
        self.sleep_dur = int(1e9 / hz)

    def _remaining(self, curr_time):
        if self.last_time > curr_time:
            self.last_time = curr_time
        elapsed = curr_time - self.last_time
        return self.sleep_dur - elapsed

    def sleep(self):
        curr_time = self.time_function()

        n = self.time_function()
        e = n + self._remaining(curr_time)
        while n < e:
            n = self.time_function()

        self.last_time = self.last_time + self.sleep_dur

        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time


class FrankaPandaEnvRosPhysics(FrankaPandaEnv):
    def __init__(self, connection_mode=p.GUI, frequency=1000., controller='position',
                 include_gripper=True, simple_model=False, object_from_sdf=False, object_from_list=False):
        with HideOutput():
            super().__init__(connection_mode=connection_mode, frequency=frequency, controller=controller,
                             include_gripper=include_gripper, simple_model=simple_model,
                             object_from_sdf=object_from_sdf, object_from_list=object_from_list)

        self.rate = Rate(frequency)

        if self.controller == 'position':
            self.current_joint_input = self.panda_robot.home_joint[:self.panda_robot.dof]
        else:
            self.current_joint_input = [0, 0, 0, 0, 0, 0, 0]
        self.current_gripper_input = True

    def simulate_step(self, joint_input, gripper_input, joint_data, tf):
        pos_reading, vel_reading, force_reading, effort_reading = self.panda_robot.get_pos_vel_force_torque()

        with joint_data.get_lock():
            joint_data_np_array = np.frombuffer(joint_data.get_obj(), dtype=np.float32).reshape((3, 9))
            joint_data_np_array[:, :len(self.panda_robot.joints)] = np.stack([pos_reading, vel_reading, effort_reading])

        self.current_gripper_input = gripper_input.value

        with joint_input.get_lock():
            self.current_joint_input = joint_input.get_obj()

        if self.controller == 'position':
            self.panda_robot.set_target_positions(self.current_joint_input)
        elif self.controller == 'velocity':
            self.panda_robot.set_target_velocities(self.current_joint_input)
        elif self.controller == 'torque':
            self.panda_robot.set_target_torques(self.current_joint_input)

        if self.include_gripper:
            if self.current_gripper_input:
                self.panda_robot.open_gripper()
            else:
                self.panda_robot.close_gripper()

        self.publish_object_tf(tf)
        self.bc.stepSimulation()
        self.rate.sleep()

    def simulation_loop(self, joint_input, gripper_input, joint_data, tf):
        while self.bc.isConnected():
            # start = time.time()
            self.simulate_step(joint_input, gripper_input, joint_data, tf)
            # print("Physics: ", 1 / (time.time() - start))
        self.bc.disconnect()
        print("Physics simulation end")

    def publish_object_tf(self, tf):
        for id in self.object_id:
            pos, orn = self.bc.getBasePositionAndOrientation(id)
            tf[str(id)] = {'translation': pos, 'rotation': orn}


class FrankaPandaEnvRosVisual(FrankaPandaEnv):
    def __init__(self, connection_mode=p.GUI, frequency=1000., controller='position',
                 include_gripper=True, simple_model=False, object_from_sdf=False, object_from_list=False):
        with HideOutput():
            super().__init__(connection_mode=connection_mode, frequency=frequency, controller=controller,
                             include_gripper=include_gripper, simple_model=simple_model,
                             object_from_sdf=object_from_sdf, object_from_list=object_from_list)

        self.rate = Rate(frequency)

        self.current_joint_state = self.panda_robot.home_joint
        if not self.include_gripper:
            self.current_joint_state = self.current_joint_state[:self.panda_robot.dof]

        self.camera_width = 640
        self.camera_height = 480
        self.camera_near = 0.02
        self.camera_far = 5.00
        self.K = np.array([[606., 0., 320.], [0., 606., 240], [0., 0., 1.]])
        self.projection_matrix = np.array([
            [2 / self.camera_width * self.K[0, 0], 0, (self.camera_width - 2 * self.K[0, 2]) / self.camera_width, 0],
            [0, 2 / self.camera_height * self.K[1, 1], (2 * self.K[1, 2] - self.camera_height) / self.camera_height, 0],
            [0, 0, (self.camera_near + self.camera_far) / (self.camera_near - self.camera_far),
             2 * self.camera_near * self.camera_far / (self.camera_near - self.camera_far)],
            [0, 0, -1, 0]]).T

    def simulate_step(self, joint_data, tf, image_rgb, image_depth):
        with joint_data.get_lock():
            joint_data_np_array = np.frombuffer(joint_data.get_obj(), dtype=np.float32).reshape((3, 9))
            self.current_joint_state = joint_data_np_array[0]
        current_joint = self.current_joint_state
        for i in range(self.panda_robot.dof):
            self.bc.resetJointState(self.panda_robot.robot_id, self.panda_robot.joints[i],
                                    targetValue=current_joint[i])

        for id in self.object_id:
            try:
                pos = tf[str(id)]['translation']
                orn = tf[str(id)]['rotation']
                self.bc.resetBasePositionAndOrientation(id, pos, orn)
            except:
                continue

        color, depth, pos, orn = self.get_image()
        with image_rgb.get_lock():
            image_rgb_np_array = np.frombuffer(image_rgb.get_obj(), dtype=np.int32).reshape((480, 640, 3))
            image_rgb_np_array[:, :] = color[:, :, :3]
        with image_depth.get_lock():
            image_depth_np_array = np.frombuffer(image_depth.get_obj(), dtype=np.float32).reshape((480, 640))
            image_depth_np_array[:, :] = depth
        tf['camera'] = {'translation': pos, 'rotation': orn}
        self.rate.sleep()

    def simulation_loop(self, joint_data, tf, image_rgb, image_depth):
        while self.bc.isConnected():
            # start = time.time()
            self.simulate_step(joint_data, tf, image_rgb, image_depth)
            # print("Visual: ", 1 / (time.time() - start))
        self.bc.disconnect()
        print("Visual simulation end")

    def get_image(self):
        # pos = [-1., 0., 0.5]
        # orn = [0.5, -0.5, 0.5, -0.5]
        camera_joint_id = 7
        joint_info = self.bc.getJointInfo(self.panda_robot.robot_id, camera_joint_id)
        parent_link = joint_info[-1]
        link_info = self.bc.getLinkState(self.panda_robot.robot_id, parent_link)
        link_pose_world = link_info[0]
        link_ori_world = link_info[1]
        pos = np.array(link_pose_world)
        orn = link_ori_world
        pos += np.array(p.getMatrixFromQuaternion(orn)).reshape((3, 3))[:, 2] * 0.2

        self.view_matrix = cvPose2BulletView(pos, orn)
        img = self.bc.getCameraImage(self.camera_width, self.camera_height, self.view_matrix,
                                     self.projection_matrix.reshape(-1).tolist(),
                                     renderer=self.bc.ER_BULLET_HARDWARE_OPENGL,
                                     flags=self.bc.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
        color = np.array(img[2])
        depth = self.camera_far * self.camera_near / (
                self.camera_far - (self.camera_far - self.camera_near) * np.array(img[3]))
        depth = np.where(depth >= self.camera_far, np.zeros_like(depth), depth)

        return color, depth, pos, orn


def cvPose2BulletView(t, q):
    """
    cvPose2BulletView gets orientation and position as used
    in ROS-TF and opencv and coverts it to the view matrix used
    in openGL and pyBullet.

    :param q: ROS orientation expressed as quaternion [qx, qy, qz, qw]
    :param t: ROS postion expressed as [tx, ty, tz]
    :return:  4x4 view matrix as used in pybullet and openGL

    """
    R = np.array(p.getMatrixFromQuaternion(q)).reshape((3, 3))

    T = np.vstack([np.hstack([R, np.array(t).reshape(3, 1)]),
                   np.array([0, 0, 0, 1])])
    # Convert opencv convention to python convention
    # By a 180 degrees rotation along X
    Tc = np.array([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]]).reshape(4, 4)

    # pybullet pse is the inverse of the pose from the ROS-TF
    T = Tc @ np.linalg.inv(T)
    # The transpose is needed for respecting the array structure of the OpenGL
    viewMatrix = T.T.reshape(16)
    return viewMatrix


def run_physics(joint_input, gripper_input, joint_data, tf,
                include_gripper, object_from_sdf, object_from_list):
    env = FrankaPandaEnvRosPhysics(connection_mode=p.GUI,
                                   frequency=2000.,
                                   controller='position',
                                   include_gripper=include_gripper,
                                   simple_model=True,
                                   object_from_sdf=object_from_sdf,
                                   object_from_list=object_from_list)

    env.simulation_loop(joint_input, gripper_input, joint_data, tf)


def run_visual(joint_data, tf, image_rgb, image_depth,
               include_gripper, object_from_sdf, object_from_list):
    env = FrankaPandaEnvRosVisual(connection_mode=p.GUI,
                                  frequency=100.,
                                  controller='position',
                                  include_gripper=include_gripper,
                                  simple_model=False,
                                  object_from_sdf=object_from_sdf,
                                  object_from_list=object_from_list)

    env.simulation_loop(joint_data, tf, image_rgb, image_depth)
