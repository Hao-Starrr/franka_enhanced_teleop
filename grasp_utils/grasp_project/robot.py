import numpy as np
from typing import List
import pybullet as p


def get_joint_limits(body_id, joint_ids):
    """Query joint limits as (lo, hi) tuple, each with length same as
    `joint_ids`."""
    joint_limits = []
    for joint_id in joint_ids:
        joint_info = p.getJointInfo(body_id, joint_id)
        joint_limit = joint_info[8], joint_info[9]
        joint_limits.append(joint_limit)
    joint_limits = np.transpose(joint_limits)  # Dx2 -> 2xD
    return joint_limits


class KUKASAKE(object):
    def __init__(self, pos, orn):
        self.robot_id: int = p.loadURDF('robots/iiwa7_mount_sake.urdf', pos, orn,
                                        useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

        n = p.getNumJoints(self.robot_id)
        for i in range(n):
            for j in range(i + 1, n):
                p.setCollisionFilterPair(self.robot_id, self.robot_id, i, j, 0)

        for i in range(n):
            p.changeDynamics(self.robot_id, i, jointLowerLimit=-1000, jointUpperLimit=1000)

        self.ee_id = 7
        self.arm_joint_ids = np.arange(7)
        self.arm_joint_limits = get_joint_limits(self.robot_id, self.arm_joint_ids)

        self.arm_rest_poses = np.deg2rad([0, -75, 0, -120, 0, 60, 0])
        # self.arm_rest_poses = np.deg2rad([0, 30, 0, -60, 0, 90, 0])

        self.gripper_z_offset: float = 0.18
        self.gripper_link_ids: List[int] = [21, 22, 24, 25]
        self.gripper_link_sign: List[float] = [-1, 0, -1, 0]
        self.gripper_link_limit: List[float] = [-0.27, 1.57075]
        self.gripper_line = [None, None, None]
        self.camera_line = [None, None, None]

        self.projectionMatrix = p.computeProjectionMatrixFOV(fov=58, aspect=1.5, nearVal=0.02, farVal=5)
        self.image_renderer = p.ER_BULLET_HARDWARE_OPENGL

        for i in range(len(self.gripper_link_ids)):
            if i != 0:
                c = p.createConstraint(self.robot_id, self.gripper_link_ids[0],
                                       self.robot_id, self.gripper_link_ids[i],
                                       jointType=p.JOINT_GEAR,
                                       jointAxis=[0, 1, 0],
                                       parentFramePosition=[0, 0, 0],
                                       childFramePosition=[0, 0, 0])
                gearRatio = -self.gripper_link_sign[0] * self.gripper_link_sign[i]
                p.changeConstraint(c, gearRatio=gearRatio, maxForce=100, erp=1)

                c = p.createConstraint(self.robot_id, self.gripper_link_ids[i],
                                       self.robot_id, self.gripper_link_ids[0],
                                       jointType=p.JOINT_GEAR,
                                       jointAxis=[0, 1, 0],
                                       parentFramePosition=[0, 0, 0],
                                       childFramePosition=[0, 0, 0])
                gearRatio = -self.gripper_link_sign[0] * self.gripper_link_sign[i]
                p.changeConstraint(c, gearRatio=gearRatio, maxForce=100, erp=1)

            gripper_link_limit = sorted([limit * self.gripper_link_sign[i] for limit in self.gripper_link_limit])

            p.changeDynamics(self.robot_id, self.gripper_link_ids[i],
                             jointLowerLimit=gripper_link_limit[0],
                             jointUpperLimit=gripper_link_limit[1])

            # Initially set the gripper as opened
            p.resetJointState(self.robot_id, self.gripper_link_ids[i],
                              self.gripper_link_limit[0] * self.gripper_link_sign[i])

    def get_joint_state(self):
        joint_positions = np.array([i[0] for i in p.getJointStates(self.robot_id, self.arm_joint_ids)])
        return joint_positions

    def reset_arm_poses(self, position):
        for rest_pose, joint_id in zip(position, self.arm_joint_ids):
            p.resetJointState(self.robot_id, joint_id, rest_pose)

    def control_arm_poses(self, position):
        p.setJointMotorControlArray(self.robot_id, self.arm_joint_ids, p.POSITION_CONTROL,
                                    targetPositions=position)

    def open_gripper(self):
        p.setJointMotorControlArray(self.robot_id, self.gripper_link_ids, p.POSITION_CONTROL,
                                    targetPositions=[i * self.gripper_link_limit[1] for i in
                                                     self.gripper_link_sign],
                                    positionGains=[1 for i in range(len(self.gripper_link_ids))])

    def close_gripper(self):
        # p.setJointMotorControlArray(self.robot_id, self.gripper_link_ids, p.VELOCITY_CONTROL,
        #                             targetVelocities=[1 for i in self.gripper_link_sign],
        #                             forces=[100 for i in range(len(self.gripper_link_ids))])
        p.setJointMotorControlArray(self.robot_id, self.gripper_link_ids, p.POSITION_CONTROL,
                                    targetPositions=[i * self.gripper_link_limit[0] for i in
                                                     self.gripper_link_sign],
                                    targetVelocities=[1 for i in self.gripper_link_sign],
                                    forces=[100 for i in range(len(self.gripper_link_ids))])

    def control_gripper(self, position):
        if position < self.gripper_link_limit[0]:
            position = self.gripper_link_limit[0]
        elif position > self.gripper_link_limit[1]:
            position = self.gripper_link_limit[1]
        p.setJointMotorControlArray(self.robot_id, self.gripper_link_ids, p.POSITION_CONTROL,
                                    targetPositions=[i * position for i in
                                                     self.gripper_link_sign],
                                    forces=[50 for i in range(len(self.gripper_link_ids))])

    def draw_gripper_frame(self):
        scale: float = 0.1
        ee_state = p.getLinkState(self.robot_id, 7)
        ee_rotation_matrix = np.array(p.getMatrixFromQuaternion(ee_state[1])).reshape((3, 3))
        gripper_state = ee_state[0] + ee_rotation_matrix[:, 2] * self.gripper_z_offset * 1
        self.gripper_line[0] = p.addUserDebugLine(lineFromXYZ=gripper_state,
                                                  lineToXYZ=gripper_state + ee_rotation_matrix[:, 0] * scale,
                                                  lineColorRGB=[1, 0, 0])
        self.gripper_line[1] = p.addUserDebugLine(lineFromXYZ=gripper_state,
                                                  lineToXYZ=gripper_state + ee_rotation_matrix[:, 1] * scale,
                                                  lineColorRGB=[0, 1, 0])
        self.gripper_line[2] = p.addUserDebugLine(lineFromXYZ=gripper_state,
                                                  lineToXYZ=gripper_state + ee_rotation_matrix[:, 2] * scale,
                                                  lineColorRGB=[0, 0, 1])

    def delete_gripper_frame(self):
        if self.gripper_line[0] is not None:
            p.removeUserDebugItem(self.gripper_line[0])
        if self.gripper_line[1] is not None:
            p.removeUserDebugItem(self.gripper_line[1])
        if self.gripper_line[2] is not None:
            p.removeUserDebugItem(self.gripper_line[2])

    def get_ee_transform(self):
        ee_state = p.getLinkState(self.robot_id, 7)  # 12, 18
        ee_T = np.eye(4)
        ee_T[:3, :3] = np.array(p.getMatrixFromQuaternion(ee_state[1])).reshape((3, 3))
        ee_T[:3, 3] = np.array(ee_state[0])
        return ee_T

    def draw_camera_frame(self):
        scale: float = 0.1
        ee_state = p.getLinkState(self.robot_id, 12)  # 12, 18
        ee_rotation_matrix = np.array(p.getMatrixFromQuaternion(ee_state[1])).reshape((3, 3))
        gripper_state = ee_state[0] + ee_rotation_matrix[:, 2] * self.gripper_z_offset * 0.0
        self.camera_line[0] = p.addUserDebugLine(lineFromXYZ=gripper_state,
                                                 lineToXYZ=gripper_state + ee_rotation_matrix[:, 0] * scale,
                                                 lineColorRGB=[1, 0, 0])
        self.camera_line[1] = p.addUserDebugLine(lineFromXYZ=gripper_state,
                                                 lineToXYZ=gripper_state + ee_rotation_matrix[:, 1] * scale,
                                                 lineColorRGB=[0, 1, 0])
        self.camera_line[2] = p.addUserDebugLine(lineFromXYZ=gripper_state,
                                                 lineToXYZ=gripper_state + ee_rotation_matrix[:, 2] * scale,
                                                 lineColorRGB=[0, 0, 1])

    def delete_camera_frame(self):
        if self.camera_line[0] is not None:
            p.removeUserDebugItem(self.camera_line[0])
        if self.camera_line[1] is not None:
            p.removeUserDebugItem(self.camera_line[1])
        if self.camera_line[2] is not None:
            p.removeUserDebugItem(self.camera_line[2])

    def get_camera_from_world_frame(self):
        # Get on-module camera (Intel Realsense D435) frame expressed in world frame
        camera_joint_id = 12
        joint_info = p.getJointInfo(self.robot_id, camera_joint_id)
        joint_pose_parent = joint_info[-3]
        joint_ori_parent = joint_info[-2]
        parent_link = joint_info[-1]
        link_info = p.getLinkState(self.robot_id, parent_link)
        link_pose_world = link_info[0]
        link_ori_world = link_info[1]
        return [joint_pose_parent, joint_ori_parent, link_pose_world, link_ori_world]

    def get_view_matrix(self):
        _, _, pos, ori = self.get_camera_from_world_frame()

        rot_matrix = p.getMatrixFromQuaternion(ori)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Initial vectors
        init_camera_vector = (1, 0, 0)  # z-axis
        init_up_vector = (0, 0, 1)  # y-axis

        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)

        view_matrix_gripper = p.computeViewMatrix(pos, pos + 0.1 * camera_vector, up_vector)
        return view_matrix_gripper

    def get_image(self):
        view_matrix_gripper = self.get_view_matrix()
        img = p.getCameraImage(640, 480, view_matrix_gripper, self.projectionMatrix, shadow=0,
                               renderer=self.image_renderer,
                               flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
        return img
