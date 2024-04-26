import rospy
from std_msgs.msg import Float32MultiArray
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from sensor_msgs.msg import JointState


class VRControlNode:
    def __init__(self):
        rospy.init_node('vr_control_node', anonymous=True)

        # 订阅righthand topic，假设这是end effector的位置信息
        self.subscriber = rospy.Subscriber(
            'righthand', Float32MultiArray, self.callback)

        self.joint_sub = rospy.Subscriber(
            '/joint', JointState, self.joint_callback)

        # 机器人模型
        self.robot = rtb.models.Panda()

        # 储存上一次的位置和时间以计算速度
        self.last_position = None
        self.last_time = None

        self.current_q = None

    def joint_callback(self, msg):
        # 这里的joints是从topic里得到的，是7个关节的角度
        self.current_q = np.array(msg.position)[0:7]

    def callback(self, data):
        # 获取当前时间和位置
        current_time = rospy.Time.now()
        current_position = np.array(data.data[:3])  # 只取xyz坐标

        if self.last_position is not None:
            # 计算时间差和位置差
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                velocity = (current_position - self.last_position) / dt

                # 计算关节速度
                J = self.robot.jacob0(self.current_q)
                dq = np.linalg.pinv(
                    J) @ np.hstack((velocity, [0, 0, 0]))  # 只考虑线性速度

                # 打印关节速度
                print("Calculated joint velocities (dq):", dq)
                # 打印ee速度
                print("Calculated end-effector velocity (v):", velocity)

        # 更新位置和时间
        self.last_position = current_position
        self.last_time = current_time

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = VRControlNode()
    node.run()
