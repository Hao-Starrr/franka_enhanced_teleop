#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import tf
from geometry_msgs.msg import Point
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
import scipy.spatial.transform


class ControlFusionNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fusion_node', anonymous=True)

        self.robot = rtb.models.Panda()

        # 创建发布者
        self.pub = rospy.Publisher(
            '/franka_physics_position_controller', Float64MultiArray, queue_size=10)

        # 创建订阅者,融合这两个dq
        self.sub_vr = rospy.Subscriber(
            '/control_from_vr', Float64MultiArray, self.callback_vr)
        self.sub_field = rospy.Subscriber(
            '/control_from_field', Float64MultiArray, self.callback_field)

        # 用来算权重的订阅者
        self.hand_sub = rospy.Subscriber(
            '/righthand', Float32MultiArray, self.hand_callback)
        self.joint_sub = rospy.Subscriber(
            '/joint', JointState, self.joint_callback)

        self.listener = tf.TransformListener()

        # callback vr and callback field
        self.dq_vr = None
        self.dq_field = None

        # initial 权重
        self.weight_field = 0.5
        self.weight_vr = 1 - self.weight_field

        # callback vr will also update these variables and decide the weight
        self.obj_ee_dist = None
        self.hand_ee_dist = None
        self.hand_ee_angle = None
        self.gravity_range = 0.5

        # update by hand_callback
        self.hand_pos = None
        self.hand_ori = None

        # update by joint_callback
        self.current_q = None
        self.current_ee_pose = None
        self.current_ee_position = None
        self.current_ee_orient = None

        self.listener.waitForTransform(
            '/2', '/ee', rospy.Time(0), rospy.Duration(2.0))

    def hand_callback(self, data):
        hand_position = np.array(data.data[0:3])  # 只取xyz坐标
        hand_orientation = self.quat2mat(data.data[3:7])  # 只取四元数
        hand_position[2] -= 0.65

        self.hand_pos = hand_position
        self.hand_ori = hand_orientation

    def quat2mat(self, q):
        rotation = scipy.spatial.transform.Rotation.from_quat(q)
        rotation_matrix = rotation.as_matrix()
        return rotation_matrix

    def joint_callback(self, msg):
        # 这里的joints是从topic里得到的，是7个关节的角度
        self.current_q = np.array(msg.position)[0:7]
        self.current_ee_pose = self.robot.fkine(self.current_q).A
        self.current_ee_position = self.current_ee_pose[0:3, 3].flatten()
        self.current_ee_orient = self.current_ee_pose[0:3, 0:3]

    def callback_field(self, data):
        self.dq_field = data.data

    def callback_vr(self, data):
        self.dq_vr = data.data
        self.update_obj_ee_dist()
        print("obj_ee_dist: ", self.obj_ee_dist)

        self.update_hand_ee_dist()
        print("hand_ee_dist: ", self.hand_ee_dist)

        self.update_hand_ee_angle()
        print("hand_ee_orient_diff: ", self.hand_ee_angle)

        self.calculate_weight()
        print("field weight: ", self.weight_field)

        self.publish_control_msg()

    def update_obj_ee_dist(self):
        try:
            # 获取frame2到ee的变换
            (trans, rot) = self.listener.lookupTransform(
                '/2', '/ee', rospy.Time(0))
            # 存储位置差
            self.obj_ee_dist = np.linalg.norm(np.array(trans))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to compute transform: %s" % str(e))

    def update_hand_ee_dist(self):
        self.hand_ee_dist = np.linalg.norm(
            self.hand_pos - self.current_ee_position)

    def update_hand_ee_angle(self):
        R_curr = self.current_ee_orient
        hand_orientation = self.hand_ori
        R_hand_in_gripper = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        hand_orientation = (R_hand_in_gripper @ hand_orientation.T).T
        R_des = hand_orientation
        self.hand_ee_angle = self.calcAngDiff(R_des, R_curr)

    def calcAngDiff(self, R_des, R_curr):
        invert_operator = -np.ones_like(R_des)
        invert_operator[0, 2] = 1
        invert_operator[1, 2] = 1
        invert_operator[2, 2] = 1
        R_des_opposite = R_des * invert_operator

        R = R_des @ R_curr.T
        trace = np.clip(np.trace(R), -1, 3)
        angle = np.arccos((trace - 1) / 2)

        R = R_des_opposite @ R_curr.T
        trace_opposite = np.clip(np.trace(R), -1, 3)
        angle_opposite = np.arccos((trace_opposite - 1) / 2)
        return min(angle, angle_opposite)

    def publish_control_msg(self):
        # 确保两个话题的数据都已接收到
        if self.dq_vr is not None and self.dq_field is not None:
            # 执行加权平均
            averaged_data = [self.weight_vr * vr + self.weight_field *
                             field for vr, field in zip(self.dq_vr, self.dq_field)]

            # 创建Float64MultiArray消息
            result_msg = Float64MultiArray()
            result_msg.data = averaged_data

            # 发布消息
            self.pub.publish(result_msg)

    def calculate_weight(self):
        if self.obj_ee_dist is not None and self.hand_ee_dist is not None:
            weight = (- 1/self.gravity_range**2 * self.obj_ee_dist**2 + 1.2)
            # weight = np.clip(weight, 0, 1)

            weight -= self.hand_ee_dist / 1.0
            # weight = np.clip(weight, 0, 1)

            weight -= (self.hand_ee_angle - 0.2) / 2.5
            weight = np.clip(weight, 0, 1)

            # test the field weight
            # if self.obj_ee_dist < self.gravity_range:
            #     weight = 1
            # else:
            #     weight = 0

            # weight = 0
            self.weight_field = weight
            self.weight_vr = 1 - weight


if __name__ == '__main__':
    try:
        node = ControlFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
