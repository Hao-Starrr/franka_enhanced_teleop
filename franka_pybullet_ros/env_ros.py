#!/usr/bin/env python
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState, Image, CameraInfo

import cv2
import numpy as np
import pybullet as p
from env import FrankaPandaEnv


def quaternion_from_matrix(R, strict_check=True):
    q = np.empty(4)
    trace = np.trace(R)
    if trace > 0.0:
        sqrt_trace = np.sqrt(1.0 + trace)
        q[3] = 0.5 * sqrt_trace
        q[0] = 0.5 / sqrt_trace * (R[2, 1] - R[1, 2])
        q[1] = 0.5 / sqrt_trace * (R[0, 2] - R[2, 0])
        q[2] = 0.5 / sqrt_trace * (R[1, 0] - R[0, 1])
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            sqrt_trace = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = 0.5 / sqrt_trace * (R[2, 1] - R[1, 2])
            q[0] = 0.5 * sqrt_trace
            q[1] = 0.5 / sqrt_trace * (R[1, 0] + R[0, 1])
            q[2] = 0.5 / sqrt_trace * (R[0, 2] + R[2, 0])
        elif R[1, 1] > R[2, 2]:
            sqrt_trace = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = 0.5 / sqrt_trace * (R[0, 2] - R[2, 0])
            q[0] = 0.5 / sqrt_trace * (R[1, 0] + R[0, 1])
            q[1] = 0.5 * sqrt_trace
            q[2] = 0.5 / sqrt_trace * (R[2, 1] + R[1, 2])
        else:
            sqrt_trace = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = 0.5 / sqrt_trace * (R[1, 0] - R[0, 1])
            q[0] = 0.5 / sqrt_trace * (R[0, 2] + R[2, 0])
            q[1] = 0.5 / sqrt_trace * (R[2, 1] + R[1, 2])
            q[2] = 0.5 * sqrt_trace
    return q


class FrankaPandaEnvRosPhysics(FrankaPandaEnv):
    def __init__(self, connection_mode=p.GUI, frequency=1000., controller='position',
                 include_gripper=True, simple_model=False, object_from_sdf=False, object_from_list=False):
        super().__init__(connection_mode=connection_mode, frequency=frequency, controller=controller,
                         include_gripper=include_gripper, simple_model=simple_model,
                         object_from_sdf=object_from_sdf, object_from_list=object_from_list)

        rospy.init_node('franka_physics', anonymous=True)
        rospy.Subscriber('franka_physics_' + self.controller +
                         '_controller', Float64MultiArray, self.get_joint_input)
        rospy.Subscriber('gripper_command', Bool, self.get_gripper_input)

        self.joint_state_pub = rospy.Publisher(
            'joint', JointState, queue_size=10)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.rate = rospy.Rate(frequency)

        if self.controller == 'position':
            self.current_joint_input = self.panda_robot.home_joint[:self.panda_robot.dof]
        else:
            self.current_joint_input = [0, 0, 0, 0, 0, 0, 0]
        self.current_gripper_input = True

        self.joint_data = JointState()
        self.joint_data.name = self.panda_robot.joint_names
        self.joint_data.header.frame_id = "franka_physics_joint"

        self.static_tf = TransformStamped()
        self.static_tf.header.frame_id = "world"

    def get_joint_input(self, reading):
        self.current_joint_input = reading.data

    def get_gripper_input(self, reading):
        self.current_gripper_input = reading.data

    def simulate_step(self):
        pos_reading, vel_reading, force_reading, effort_reading = self.panda_robot.get_pos_vel_force_torque()
        self.joint_data.position = pos_reading
        self.joint_data.velocity = vel_reading
        self.joint_data.effort = effort_reading
        self.joint_data.header.stamp = rospy.get_rostime()
        self.joint_state_pub.publish(self.joint_data)

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

        self.publish_object_tf()
        self.bc.stepSimulation()
        self.rate.sleep()

    def simulation_loop(self):
        import time
        while not rospy.is_shutdown():
            start = time.time()
            self.simulate_step()
            print("Physics: ", 1 / (time.time() - start))
        self.bc.disconnect()
        print("Physics simulation end")

    def publish_object_tf(self):
        self.static_tf.header.stamp = rospy.Time.now()
        for id in self.object_id:
            pos, orn = self.bc.getBasePositionAndOrientation(id)
            self.static_tf.child_frame_id = str(id)
            self.static_tf.transform.translation.x = pos[0]
            self.static_tf.transform.translation.y = pos[1]
            self.static_tf.transform.translation.z = pos[2]
            self.static_tf.transform.rotation.x = orn[0]
            self.static_tf.transform.rotation.y = orn[1]
            self.static_tf.transform.rotation.z = orn[2]
            self.static_tf.transform.rotation.w = orn[3]
            self.tf_broadcaster.sendTransform(self.static_tf)


class FrankaPandaEnvRosVisual(FrankaPandaEnv):
    def __init__(self, connection_mode=p.GUI, frequency=1000., controller='position',
                 include_gripper=True, simple_model=False, object_from_sdf=False, object_from_list=False):
        super().__init__(connection_mode=connection_mode, frequency=frequency, controller=controller,
                         include_gripper=include_gripper, simple_model=simple_model,
                         object_from_sdf=object_from_sdf, object_from_list=object_from_list)

        rospy.init_node('franka_visual', anonymous=True)

        self.color_image_publisher = rospy.Publisher(
            '/camera/rgb/image_raw', Image, queue_size=10)
        self.depth_image_publisher = rospy.Publisher(
            '/camera/depth/image_raw', Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher(
            '/camera/rgb/camera_info', CameraInfo, queue_size=10)
        self.mask_image_publisher = rospy.Publisher(
            '/camera/mask/image_raw', Image, queue_size=10)

        # ###############################################################
        self.color_view_publisher = rospy.Publisher(
            '/view/rgb/image_raw', Image, queue_size=10)
        self.view_info_publisher = rospy.Publisher(
            '/view/rgb/camera_info', CameraInfo, queue_size=10)
        #################################################################
        self.cv_bridge = CvBridge()

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.joint_state_sub = rospy.Subscriber(
            'joint', JointState, self.get_joint_state)
        self.rate = rospy.Rate(frequency)

        self.current_joint_state = self.panda_robot.home_joint
        if not self.include_gripper:
            self.current_joint_state = self.current_joint_state[:self.panda_robot.dof]

        self.camera_width = 640
        self.camera_height = 480
        self.camera_near = 0.02
        self.camera_far = 5.00
        self.K = np.array([[606., 0., 320.], [0., 606., 240], [0., 0., 1.]])
        self.projection_matrix = np.array([
            [2 / self.camera_width * self.K[0, 0], 0,
                (self.camera_width - 2 * self.K[0, 2]) / self.camera_width, 0],
            [0, 2 / self.camera_height * self.K[1, 1],
                (2 * self.K[1, 2] - self.camera_height) / self.camera_height, 0],
            [0, 0, (self.camera_near + self.camera_far) / (self.camera_near - self.camera_far),
             2 * self.camera_near * self.camera_far / (self.camera_near - self.camera_far)],
            [0, 0, -1, 0]]).T

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.camera_width
        self.camera_info_msg.height = self.camera_height
        self.camera_info_msg.K = self.K.reshape(-1).astype(float).tolist()
        self.camera_info_msg.header.frame_id = "actual_camera"
        # self.camera_info_msg.header.stamp = rospy.get_rostime()
        self.camera_info_msg.D = [0, 0, 0, 0, 0]
        self.camera_info_msg.P = self.projection_matrix[:3, :].reshape(
            -1).tolist()
        self.camera_info_msg.distortion_model = "plumb_bob"

        ##################################################################
        self.view_info_msg = CameraInfo()
        self.view_info_msg.width = self.camera_width
        self.view_info_msg.height = self.camera_height
        self.view_info_msg.K = self.K.reshape(-1).astype(float).tolist()
        self.view_info_msg.header.frame_id = "actual_view"
        self.view_info_msg.D = [0, 0, 0, 0, 0]
        self.view_info_msg.P = self.projection_matrix[:3, :].reshape(
            -1).tolist()
        self.view_info_msg.distortion_model = "plumb_bob"
        ####################################################################

        self.static_tf = TransformStamped()
        self.static_tf.header.frame_id = "world"

        # self.panda_robot 可以用

    def get_joint_state(self, reading):
        self.current_joint_state = reading.position

    def simulate_step(self):
        current_joint = self.current_joint_state
        for i in range(len(current_joint)):
            self.bc.resetJointState(
                self.panda_robot.robot_id, self.panda_robot.joints[i], targetValue=current_joint[i])

        for id in self.object_id:
            try:
                trans = self.tfBuffer.lookup_transform(
                    'world', str(id), rospy.Time())
                pos = [trans.transform.translation.x,
                       trans.transform.translation.y, trans.transform.translation.z]
                orn = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                       trans.transform.rotation.w]
                self.bc.resetBasePositionAndOrientation(id, pos, orn)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

        color, depth, mask, cam_pos, cam_orn, ee_pos, ee_orn = self.get_image()
        stamp = rospy.Time.now()

        image = self.cv_bridge.cv2_to_imgmsg(color)
        image.header.frame_id = "color_camera"
        image.header.stamp = stamp
        self.color_image_publisher.publish(image)

        image = self.cv_bridge.cv2_to_imgmsg(depth)
        image.header.frame_id = "actual_camera"
        image.header.stamp = stamp
        self.depth_image_publisher.publish(image)

        image = self.cv_bridge.cv2_to_imgmsg(mask)
        image.header.frame_id = "actual_camera"
        image.header.stamp = stamp
        self.mask_image_publisher.publish(image)

        self.camera_info_msg.header.stamp = stamp
        self.camera_info_publisher.publish(self.camera_info_msg)

        self.static_tf.header.stamp = stamp
        self.static_tf.child_frame_id = "actual_camera"
        self.static_tf.transform.translation.x = cam_pos[0]
        self.static_tf.transform.translation.y = cam_pos[1]
        self.static_tf.transform.translation.z = cam_pos[2]
        self.static_tf.transform.rotation.x = cam_orn[0]
        self.static_tf.transform.rotation.y = cam_orn[1]
        self.static_tf.transform.rotation.z = cam_orn[2]
        self.static_tf.transform.rotation.w = cam_orn[3]
        self.tf_broadcaster.sendTransform(self.static_tf)

        ##############################################################
        # 获取视图图像及其相机位置和方向
        view_color, view_pos, view_orn = self.get_view()
        # 发布视图图像
        view_image = self.cv_bridge.cv2_to_imgmsg(view_color)
        view_image.header.frame_id = "actual_view"
        view_image.header.stamp = stamp
        self.color_view_publisher.publish(view_image)
        # 发布视图相机信息
        self.view_info_msg.header.stamp = stamp
        self.view_info_publisher.publish(self.view_info_msg)
        ##############################################################

        self.static_tf.header.stamp = stamp
        self.static_tf.child_frame_id = "ee"
        self.static_tf.transform.translation.x = ee_pos[0]
        self.static_tf.transform.translation.y = ee_pos[1]
        self.static_tf.transform.translation.z = ee_pos[2]
        self.static_tf.transform.rotation.x = ee_orn[0]
        self.static_tf.transform.rotation.y = ee_orn[1]
        self.static_tf.transform.rotation.z = ee_orn[2]
        self.static_tf.transform.rotation.w = ee_orn[3]
        self.tf_broadcaster.sendTransform(self.static_tf)
        self.rate.sleep()

    def simulation_loop(self):
        import time
        while not rospy.is_shutdown():
            start = time.time()
            self.simulate_step()
            print("Visual: ", 1 / (time.time() - start))
        self.bc.disconnect()
        print("Visual simulation end")

    def get_image(self):
        # pos = [-1., 0., 0.5]
        # orn = [0.5, -0.5, 0.5, -0.5]
        camera_joint_id = 7
        joint_info = self.bc.getJointInfo(
            self.panda_robot.robot_id, camera_joint_id)
        parent_link = joint_info[0]
        link_info = self.bc.getLinkState(
            self.panda_robot.robot_id, parent_link)
        link_pose_world = link_info[0]
        link_ori_world = link_info[1]
        pos = np.array(link_pose_world)
        orn = link_ori_world
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape((3, 3))
        ee_T = np.eye(4)
        ee_T[:3, :3] = R
        ee_T[:3, 3] = pos
        ee_cam_T = np.array([[0.7061203, -0.7078083, -0.0200362, 0.00291476],
                             [0.7080078, 0.7061898, 0.0045781, -0.04739189],
                             [0.0109089, -0.0174185, 0.9997888, 0.0643254],
                             [0., 0., 0., 1.]])
        cam_T = ee_T @ ee_cam_T
        cam_pos = cam_T[:3, 3]
        cam_orn = quaternion_from_matrix(cam_T[:3, :3])

        # self.bc.addUserDebugLine(cam_pos, cam_pos + cam_T[:3,0] * 0.1, lineColorRGB=[1, 0, 0])
        # self.bc.addUserDebugLine(cam_pos, cam_pos + cam_T[:3,1] * 0.1, lineColorRGB=[0, 1, 0])
        # self.bc.addUserDebugLine(cam_pos, cam_pos + cam_T[:3,2] * 0.1, lineColorRGB=[0, 0, ])

        self.view_matrix = cvPose2BulletView(cam_pos, cam_orn)
        img = self.bc.getCameraImage(self.camera_width, self.camera_height, self.view_matrix,
                                     self.projection_matrix.reshape(
                                         -1).tolist(),
                                     renderer=self.bc.ER_BULLET_HARDWARE_OPENGL,
                                     flags=self.bc.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)

        color = cv2.cvtColor(np.array(img[2]), cv2.COLOR_RGB2BGR)
        # depth = self.camera_far * self.camera_near / (
        #     self.camera_far - (self.camera_far - self.camera_near) * np.array(img[3]))
        depth = np.array(img[3])
        # depth = np.where(depth >= self.camera_far, np.zeros_like(depth), depth)
        #####
        mask = np.array(img[4]).astype(np.uint16)
        #####
        return color, depth, mask, cam_pos, cam_orn, pos, orn

    #####################################################################
    def get_view(self):

        cam_T = np.array([[0.7061203, -0.7078083, -0.0200362, 0.00291476],
                          [0.7080078, 0.7061898, 0.0045781, -0.04739189],
                          [0.0109089, -0.0174185, 0.9997888, 0.0643254],
                          [0., 0., 0., 1.]])

        view_pos = cam_T[:3, 3]
        view_orn = quaternion_from_matrix(cam_T[:3, :3])

        # 将视角位置和方向转换成仿真环境中的视图矩阵
        view_matrix = cvPose2BulletView(view_pos, view_orn)

        # 获取相机图像
        img = self.bc.getCameraImage(self.camera_width, self.camera_height, view_matrix,
                                     self.projection_matrix.reshape(
                                         -1).tolist(),
                                     renderer=self.bc.ER_BULLET_HARDWARE_OPENGL,
                                     flags=self.bc.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
        # img[2] is the color image, img[3] is the depth image
        # img[4] is the segmentation mask

        # 提取彩色图像，并将其从RGB转换为BGR格式
        color = cv2.cvtColor(np.array(img[2]), cv2.COLOR_RGB2BGR)

        # 返回彩色图像、视角位置和视角方向
        return color, view_pos, view_orn
    #######################################################################


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
