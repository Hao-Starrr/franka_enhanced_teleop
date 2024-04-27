#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import tf
from geometry_msgs.msg import Point
import numpy as np


class DataFusionNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fusion_node', anonymous=True)

        # 创建发布者
        self.pub = rospy.Publisher(
            '/franka_physics_position_controller', Float64MultiArray, queue_size=10)

        # 创建订阅者
        self.sub_vr = rospy.Subscriber(
            '/control_from_vr', Float64MultiArray, self.callback_vr)
        self.sub_field = rospy.Subscriber(
            '/control_from_field', Float64MultiArray, self.callback_field)

        self.listener = tf.TransformListener()

        # 初始化变量以存储数据
        self.data_vr = None
        self.data_field = None

        # 权重，根据实际需求调整
        self.weight_field = 0.5
        self.weight_vr = 1 - self.weight_field

        self.dist = None
        self.gravity_range = 0.5

        self.listener.waitForTransform(
            '/2', '/ee', rospy.Time(0), rospy.Duration(2.0))

    def callback_vr(self, data):
        self.data_vr = data.data
        self.update_distance()
        print("distance: ", self.dist)

        self.calculate_weight()
        print("weight: ", self.weight_field)

        self.try_publish()

    def callback_field(self, data):
        self.data_field = data.data
        self.try_publish()

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

    def update_distance(self):
        try:
            # # 等待tf数据，可以调整超时时间
            # self.listener.waitForTransform(
            #     '/2', '/ee', rospy.Time(0), rospy.Duration(2.0))
            # 获取frame2到ee的变换
            (trans, rot) = self.listener.lookupTransform(
                '/2', '/ee', rospy.Time(0))
            # 存储位置差
            self.dist = np.linalg.norm(np.array(trans))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to compute transform: %s" % str(e))

    def calculate_weight(self):
        # 使用ReLU函数类似的处理来计算weight
        # weight = (self.gravity_range - self.dist) / self.gravity_range

        weight = (- 1/self.gravity_range**2 * self.dist**2 + 1)
        weight = np.clip(weight, 0, 1)
        # if self.dist < self.gravity_range:
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
