from std_msgs.msg import Header, Float64MultiArray
import rospy
from std_msgs.msg import Float32MultiArray
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from sensor_msgs.msg import JointState


class VRControlNode:
    def __init__(self):
        rospy.init_node('vr_control_node', anonymous=True)

        # 机器人模型
        self.robot = rtb.models.Panda()

        # 订阅righthand topic，假设这是end effector的位置信息
        self.subscriber = rospy.Subscriber(
            '/righthand', Float32MultiArray, self.callback)

        self.joint_sub = rospy.Subscriber(
            '/joint', JointState, self.joint_callback)

        self.command_pub = rospy.Publisher(
            '/control_from_vr', Float64MultiArray, queue_size=1)

        # 储存上一次的位置和时间以计算速度

        self.current_q = None
        self.current_ee_pose = None
        self.current_ee_position = None

        self.q_max = np.array(
            [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.q_min = np.array(
            [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.q_center = self.q_min + (self.q_max - self.q_min) / 2

    def joint_callback(self, msg):
        # 这里的joints是从topic里得到的，是7个关节的角度
        self.current_q = np.array(msg.position)[0:7]
        self.current_ee_pose = self.robot.fkine(self.current_q).A
        self.current_ee_position = self.current_ee_pose[0:3, 3].flatten()

    def IK_velocity(self, q_in, v_in, omega_in):

        dq = np.zeros((1, 7))

        v_in = v_in.reshape((3, 1))
        omega_in = omega_in.reshape((3, 1))
        V = np.vstack((v_in, omega_in))

        J = self.robot.jacob0(q_in)

        # we should not just assign nan as 0, because nan means no constrains
        # we can just ignore these rows

        # 主任务处理
        constrained_rows = ~np.isnan(V)
        J_constrained = J[constrained_rows.reshape(-1)]
        V_constrained = V[constrained_rows]

        J_pseudo_inverse = np.linalg.pinv(J_constrained)
        dq_primary = np.dot(J_pseudo_inverse, V_constrained)

        # 次要任务：关节位置中心化
        dq_secondary = 0.05 * (self.q_center - q_in)  # 选择一个合适的比例因子

        # 投影次要任务以防止干扰主任务
        I = np.eye(7)
        projection_matrix = I - np.dot(J_pseudo_inverse, J_constrained)
        dq_secondary_projected = np.dot(projection_matrix, dq_secondary)

        # 合并主任务和次要任务的速度命令
        dq = dq_primary + dq_secondary_projected

        return dq

    def callback(self, data):
        # 获取当前时间和位置
        hand_position = np.array(data.data[:3])  # 只取xyz坐标
        hand_position = self.swap_axis(hand_position)
        hand_position[2] -= 0.65

        # 计算位置差
        velocity = hand_position - self.current_ee_position
        omega = np.array([np.nan, np.nan, np.nan])

        dq = self.IK_velocity(self.current_q, velocity, omega)

        # 打印关节速度
        print("Calculated joint velocities (dq):", dq)
        # 打印ee速度
        print("Calculated end-effector velocity (v):", velocity)

        q_temp = self.current_q + dq.flatten() * 0.5

        publish_msg = Float64MultiArray()
        publish_msg.data = q_temp.tolist()
        # publish_msg.data[6] += 1.57
        self.command_pub.publish(publish_msg)

    def swap_axis(self, coordinate):
        x = coordinate[2]
        y = -coordinate[0]
        z = coordinate[1]
        coordinate = np.array([x, y, z]).flatten()

        return coordinate


if __name__ == '__main__':
    node = VRControlNode()
    rospy.spin()
