
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from std_msgs.msg import Float64


# 输入ee xyz和pinch state 0/1
# 输出 ee 速度
# q dot

class VRControlNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('vr_control_node')

        # 创建发布者，假设消息类型为Float64
        self.velocity_publisher = rospy.Publisher(
            '/ee_velocity', Float64, queue_size=10)

        # 创建订阅者，订阅VR设备的数据
        self.position_subscriber = rospy.Subscriber(
            '/vr_position', Point, self.position_callback)
        self.pinch_subscriber = rospy.Subscriber(
            '/vr_pinch_state', Int32, self.pinch_callback)

        # 初始化变量
        self.current_position = Point()
        self.current_pinch_state = 0

    def position_callback(self, msg):
        # 更新当前位置
        self.current_position = msg
        # 计算并发布速度
        self.compute_and_publish_velocity()

    def pinch_callback(self, msg):
        # 更新当前pinch状态
        self.current_pinch_state = msg.data
        # 计算并发布速度
        self.compute_and_publish_velocity()

    def compute_and_publish_velocity(self):
        # 计算速度的逻辑
        velocity = self.calculate_velocity(
            self.current_position, self.current_pinch_state)
        # 发布速度
        self.velocity_publisher.publish(velocity)

    def calculate_velocity(self, position, pinch_state):
        # 这里是计算逻辑，你可以根据需求进行实现
        # 示例：返回一个基于位置和pinch状态的简单计算结果
        return Float64(data=position.x + position.y + position.z + pinch_state)


if __name__ == '__main__':
    node = VRControlNode()
    rospy.spin()
