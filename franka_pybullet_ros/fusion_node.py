#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import tf
from geometry_msgs.msg import Point
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb


class DataFusionNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fusion_node', anonymous=True)
        self.robot = rtb.models.Panda()

        # 创建发布者
        self.pub = rospy.Publisher(
            '/franka_physics_position_controller', Float64MultiArray, queue_size=10)

        # 创建订阅者
        self.sub_vr = rospy.Subscriber(
            '/control_from_vr', Float64MultiArray, self.callback_vr)
        self.sub_field = rospy.Subscriber(
            '/control_from_field', Float64MultiArray, self.callback_field)

        self.subscriber = rospy.Subscriber(
            '/righthand', Float32MultiArray, self.callback_hand)

        self.joint_sub = rospy.Subscriber(
            '/joint', JointState, self.joint_callback)

        self.listener = tf.TransformListener()

        # 初始化变量以存储数据
        self.data_vr = None
        self.data_field = None

        # 权重，根据实际需求调整
        self.weight_field = 0.5
        self.weight_vr = 1 - self.weight_field

        self.obj_ee_dist = None
        self.gravity_range = 0.5

        self.hand_pos = None
        self.hand_ee_dist = None

        self.current_q = None
        self.current_ee_pose = None
        self.current_ee_position = None

        self.listener.waitForTransform(
            '/2', '/ee', rospy.Time(0), rospy.Duration(2.0))

    def callback_vr(self, data):
        self.data_vr = data.data
        self.update_obj_ee_dist()
        print("obj_ee_dist: ", self.obj_ee_dist)

        self.update_hand_ee_dist()
        print("hand_ee_dist: ", self.hand_ee_dist)

        self.calculate_weight()
        print("weight: ", self.weight_field)

        self.try_publish()

    def callback_field(self, data):
        self.data_field = data.data
        self.try_publish()

    def callback_hand(self, data):
        hand_position = np.array(data.data[:3])  # 只取xyz坐标
        hand_position = self.swap_axis(hand_position)
        hand_position[2] -= 0.65

        self.hand_pos = hand_position

    def swap_axis(self, coordinate):
        x = coordinate[2]
        y = -coordinate[0]
        z = coordinate[1]
        coordinate = np.array([x, y, z]).flatten()

        return coordinate

    def joint_callback(self, msg):
        # 这里的joints是从topic里得到的，是7个关节的角度
        self.current_q = np.array(msg.position)[0:7]
        self.current_ee_pose = self.robot.fkine(self.current_q).A
        self.current_ee_position = self.current_ee_pose[0:3, 3].flatten()

    def try_publish(self):
        # 确保两个话题的数据都已接收到
        if self.data_vr is not None and self.data_field is not None:
            # 执行加权平均
            averaged_data = [self.weight_vr * vr + self.weight_field *
                             field for vr, field in zip(self.data_vr, self.data_field)]

            # 创建Float64MultiArray消息
            result_msg = Float64MultiArray()
            result_msg.data = averaged_data

            # 发布消息
            self.pub.publish(result_msg)

            # 可选：清除数据
            # self.data_vr = None
            # self.data_field = None

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

    def calculate_weight(self):
        # 使用ReLU函数类似的处理来计算weight
        # weight = (self.gravity_range - self.obj_ee_dist) / self.gravity_range

        weight = (- 1/self.gravity_range**2 * self.obj_ee_dist**2 + 1)
        weight = np.clip(weight, 0, 1)

        weight -= self.hand_ee_dist / 1.0
        weight = np.clip(weight, 0, 1)
        # if self.obj_ee_dist < self.gravity_range:
        #     weight = 1
        # else:
        #     weight = 0

        self.weight_field = weight
        self.weight_vr = 1 - weight


if __name__ == '__main__':
    try:
        node = DataFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
