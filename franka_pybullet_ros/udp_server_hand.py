from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray
import rospy
import socket
import struct
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import geometry_msgs.msg


def start_udp_server_ros(ip, port):
    # Initialize the ROS node
    rospy.init_node('udp_to_ros_node', anonymous=True)

    # Create ROS publishers for the two hands
    pub_lefthand = rospy.Publisher(
        '/lefthand', Float32MultiArray, queue_size=10)
    pub_righthand = rospy.Publisher(
        '/righthand', Float32MultiArray, queue_size=10)

    br = tf2_ros.TransformBroadcaster()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((ip, port))
    print(f"Listening on {ip}:{port}")

    try:
        while not rospy.is_shutdown():
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes

            if len(data) == 33:

                unpack_format = '<ifffffff?'
                id, x, y, z, ox, oy, oz, ow, isPinching = struct.unpack(
                    unpack_format, data)
                print(
                    f"Received ID: {id}, Position: ({x}, {y}, {z}), Orientation: ({ox}, {oy}, {oz}, {ow}), Pinching: {isPinching} from {addr}")

                # flip the coordinate system
                x, y, z = axis_swap(np.array([x, y, z]))
                ox, oy, oz = axis_swap(np.array([ox, oy, oz]))
                # Prepare the message for ROS
                msg = Float32MultiArray()
                msg.data = [x, y, z, ox, oy, oz, -ow, float(isPinching)]

                # Publish to the correct topic based on ID
                if id == 0:  # Assuming ID 0 for lefthand, 1 for righthand
                    pub_lefthand.publish(msg)
                elif id == 1:
                    pub_righthand.publish(msg)

                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "world"
                t.child_frame_id = "e"
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.3
                t.transform.rotation.x = ox
                t.transform.rotation.y = oy
                t.transform.rotation.z = oz
                t.transform.rotation.w = -ow
                br.sendTransform(t)

            else:
                print(
                    f"Error: Expected {33} bytes but received {len(data)}")
                print(f"Received message: {data} from {addr}")

    except KeyboardInterrupt:
        print("UDP Server is closing")
        sock.close()


def axis_swap(coordinate):
    x = coordinate[2]
    y = -coordinate[0]
    z = coordinate[1]
    return x, y, z


if __name__ == "__main__":
    IP_ADDRESS = "0.0.0.0"  # Listen on all available IPs
    PORT = 7777  # The port number to listen on
    start_udp_server_ros(IP_ADDRESS, PORT)
