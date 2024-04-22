import rospy
import tf
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, Float64MultiArray
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import trimesh
import copy

import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import open3d as o3d

from grasp_sampler import GraspSampler
from riem_gmm import SE3GMM

import torch
import pytorch_kinematics as pk
import pytransform3d.trajectories

import roboticstoolbox as rtb
from spatialmath import SE3


def get_projection_matrix():
    p_matrix = p.computeProjectionMatrixFOV(
        fov=58, aspect=1.5, nearVal=0.02, farVal=5)
    return p_matrix


def get_view_matrix(pos, ori):

    rot_matrix = p.getMatrixFromQuaternion(ori)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    # Initial vectors
    # init_camera_vector = (1, 0, 0)  # z-axis
    # init_up_vector = (0, 0, 1)  # y-axis

    # 这里应该在定义相机是怎么安装在手上的
    init_camera_vector = (0, 0, 1)  # z-axis
    init_up_vector = (0, -1, 0)  # y-axis

    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)

    view_matrix_gripper = p.computeViewMatrix(
        pos, pos + 0.1 * camera_vector, up_vector)
    return view_matrix_gripper


def get_point_cloud(width, height, depth, segmentation, view_matrix, proj_matrix, object_id):

    # print(depth.shape)
    # print(segmentation.shape)

    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z, s = x.reshape(
        -1), y.reshape(-1), depth.reshape(-1), segmentation.reshape(-1)
    h = np.ones_like(z)

    pixels = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    pixels = pixels[z < 0.999]
    s = s[z < 0.999]

    pixels = pixels[s == object_id]

    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3: 4]
    points = points[:, :3]

    return points


def draw_grasp_poses(pub, H, color=[1, 0, 0]):

    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.01  # Line width
    marker.color = ColorRGBA(color[0], color[1], color[2], 1.0)  # RGBA color
    marker.pose.orientation.w = 1.0

    # Define gripper geometry relative to the base link
    gripper_vertices = np.array([
        [0.041, 0, 0.0659999996, 1],
        [0.041, 0, 0.112169998, 1],
        [-0.041, 0, 0.0659999996, 1],
        [-0.041, 0, 0.112169998, 1],
        [0, 0, 0, 1],
        [0, 0, 0.0659999996, 1]
    ])
    gripper_edges = np.array([[4, 5], [0, 2], [0, 1], [2, 3]])

    for i in range(H.shape[0]):
        gripper_points = (H[i] @ gripper_vertices.T).T[:, :3]
        for u, v in gripper_edges:
            p1 = Point(gripper_points[u][0],
                       gripper_points[u][1], gripper_points[u][2])
            p2 = Point(gripper_points[v][0],
                       gripper_points[v][1], gripper_points[v][2])
            marker.points.append(p1)
            marker.points.append(p2)

    pub.publish(marker)

# Define the class for grasp node


class GraspNode:
    def __init__(self):
        rospy.init_node('grasp_node', anonymous=True)

        self.depth_sub = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.depth_callback)
        self.mask_sub = rospy.Subscriber(
            '/camera/mask/image_raw', Image, self.mask_callback)
        self.joint_sub = rospy.Subscriber(
            '/joint', JointState, self.joint_callback)

        self.command_pub = rospy.Publisher(
            '/franka_physics_position_controller', Float64MultiArray, queue_size=1)

        self.viz_pub = rospy.Publisher('/grasp_poses', Marker, queue_size=10)

        self.depth_image = None
        self.mask_image = None
        self.joints = None

        self.tf_listener = tf.TransformListener()
        self.frame_id = 'actual_camera'  # Change to your camera frame's name
        self.cv_bridge = CvBridge()

        self.pcl_pub = rospy.Publisher(
            '/camera/point_cloud', PointCloud2, queue_size=2)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def mask_callback(self, msg):
        try:
            self.mask_image = self.cv_bridge.imgmsg_to_cv2(
                msg)  # 或者 "16UC1" 取决于您的数据
        except CvBridgeError as e:
            rospy.logerr(e)

    def joint_callback(self, msg):
        # 这里的joints是从topic里得到的，是7个关节的角度
        self.joints = np.array(msg.position)

    def wait_for_messages(self):
        rospy.loginfo("Waiting for messages...")
        while not rospy.is_shutdown() and (self.depth_image is None or self.mask_image is None):
            rospy.sleep(0.1)
        rospy.loginfo("Received all messages.")

    def estimate_grasp_pose(self):
        self.tf_listener.waitForTransform(
            '/world', self.frame_id, rospy.Time(0), rospy.Duration(2.0))
        (trans, rot) = self.tf_listener.lookupTransform(
            '/world', self.frame_id, rospy.Time(0))

        object_id = 2  # Change to your object's id
        self.points = get_point_cloud(640, 480, self.depth_image, self.mask_image, get_view_matrix(trans, rot),
                                      get_projection_matrix(), object_id)

        # print(self.points[0:10, :])

        ################# sample grasps from point cloud #######################
        model = GraspSampler()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd = pcd.voxel_down_sample(voxel_size=0.01)
        points = np.asarray(pcd.points)
        points, H, w = model.sampler(points)
        print("point cloud shape: ", points.shape)

        # 已经在rviz里画了,不用这个了
        # draw_point_cloud(self.points)

        ##################### choose the grasp pose #############################
        # # reachability
        # gmm_reach = SE3GMM()
        # parameter = np.load('gmm_reachability.npz')
        # gmm_reach.mu = parameter['mu']
        # gmm_reach.sigma = parameter['sigma']
        # gmm_reach.pi = parameter['pi']

        # print("H shape: ", H.shape)
        # H_prob = gmm_reach.eval(H)
        # H = H[H_prob > 0.0005733336724437366]

        # # grasp direction filter
        # H = H[H[:, 2, 3] > 0.05]

        print("final grasp H shape: ", H.shape)
        draw_grasp_poses(self.viz_pub, H, color=[1, 0., 0.])

        ##################### fit the distribution, then we can take the gradient ##################
        self.gmm_grasp = SE3GMM()
        H_sample = self.gmm_grasp.fit(H, n_clusters=min(
            8, H.shape[0]), n_iterations=10)

        ####################### then use self.gmm_grasp ##################

    def control(self):  # not debug

        device = "cpu"
        dtype = torch.float32
        chain = pk.build_serial_chain_from_urdf(
            open("model_description/panda.urdf").read(), "panda_hand_tcp")
        chain = chain.to(dtype=dtype, device=device)

        # joint state is from topic
        q_orig = self.joints

        # 增加维度从7变成1x7
        q_temp = torch.tensor(self.joints, dtype=dtype,
                              device=device, requires_grad=False)[None, :]
        with torch.inference_mode():
            while not rospy.is_shutdown():
                # 算出ee pose
                m = chain.forward_kinematics(
                    q_temp, end_only=True).get_matrix()
                # 换成指数坐标
                T = pytransform3d.trajectories.exponential_coordinates_from_transforms(
                    m[0].numpy())

                # 求梯度(指数坐标里的速度)
                grad = self.gmm_grasp.grad(T)
                if np.linalg.norm(grad) > 1e1:
                    grad = grad / np.linalg.norm(grad) * 1e1
                # 梯度也在指数坐标里,所以要在这个坐标里加上速度
                T_new = copy.deepcopy(T)
                T_new[:3] = T[:3] + 1e-3 * grad[:3]
                T_new[3:] = T[3:] + 1e-3 * grad[3:]
                # 转换回矩阵
                T_new = torch.tensor(pytransform3d.trajectories.transforms_from_exponential_coordinates(T_new),
                                     dtype=dtype)[None, :, :]

                # 计算位置差
                pos = T_new[:, :3, 3] - m[:, :3, 3]
                # 计算姿态差
                rot = pk.matrix_to_axis_angle(
                    T_new[:, :3, :3] @ m[:, :3, :3].transpose(1, 2))
                ee_e = torch.cat([pos, rot], dim=1)

                # 此时的J
                J = chain.jacobian(q_temp)

                # q velocity
                q_transpose = J.transpose(1, 2)
                q_dot = (q_transpose @ torch.linalg.solve(J @
                                                          q_transpose, ee_e[:, :, None]))[:, :, 0]
                q_temp += q_dot

                q_norm = torch.linalg.norm(q_dot)
                if q_norm < 0.005:
                    break

                publish_msg = Float64MultiArray()
                publish_msg.data = q_temp[0].tolist()
                self.command_pub.publish(publish_msg)

                # p.stepSimulation()
                # time.sleep(0.01)

    def publish_point_cloud(self, points):
        # 创建 PointCloud2 消息头
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'

        # 创建点云数据（这里假设 points 是 Nx3 的 numpy 数组）
        point_cloud = pc2.create_cloud_xyz32(header, points)

        # 发布点云
        self.pcl_pub.publish(point_cloud)


if __name__ == '__main__':
    try:
        gn = GraspNode()
        gn.wait_for_messages()
        # 此时您有了所有初始化时的消息，可以进行处理
        gn.estimate_grasp_pose()
        # gn.control()

        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            gn.publish_point_cloud(gn.points)
            r.sleep()

        # gn.points is what we want

    except rospy.ROSInterruptException:
        pass
