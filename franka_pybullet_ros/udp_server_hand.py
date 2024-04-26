from std_msgs.msg import Float32MultiArray
import rospy
import socket
import struct


def start_udp_server(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((ip, port))
    print(f"Listening on {ip}:{port}")

    try:
        while True:
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes

            if len(data) == 17:
                # Unpack data
                # Assuming data structure: int, float, float, float, bool
                # Format: 'i' for int, 'fff' for three floats, '?' for bool
                id, x, y, z, isPinching = struct.unpack('<ifff?', data)
                print(
                    f"Received ID: {id}, Position: ({x}, {y}, {z}), Pinching: {isPinching} from {addr}")
            else:
                print(f"Error: Expected 17 bytes but received {len(data)}")
                print(f"Received message: {data} from {addr}")

    except KeyboardInterrupt:
        print("UDP Server is closing")
        sock.close()


def start_udp_server_ros(ip, port):
    # Initialize the ROS node
    rospy.init_node('udp_to_ros_node', anonymous=True)

    # Create ROS publishers for the two hands
    pub_lefthand = rospy.Publisher(
        'lefthand', Float32MultiArray, queue_size=10)
    pub_righthand = rospy.Publisher(
        'righthand', Float32MultiArray, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind((ip, port))
    print(f"Listening on {ip}:{port}")

    try:
        while not rospy.is_shutdown():
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes

            if len(data) == 17:
                # Unpack data
                # Assuming data structure: int, float, float, float, bool
                id, x, y, z, isPinching = struct.unpack('<ifff?', data)
                print(
                    f"Received ID: {id}, Position: ({x}, {y}, {z}), Pinching: {isPinching} from {addr}")

                # Prepare the message for ROS
                msg = Float32MultiArray()
                msg.data = [x, y, z, float(isPinching)]

                # Publish to the correct topic based on ID
                if id == 0:  # Assuming ID 0 for lefthand, 1 for righthand
                    pub_lefthand.publish(msg)
                elif id == 1:
                    pub_righthand.publish(msg)
            else:
                print(f"Error: Expected 17 bytes but received {len(data)}")
                print(f"Received message: {data} from {addr}")

    except KeyboardInterrupt:
        print("UDP Server is closing")
        sock.close()


if __name__ == "__main__":
    IP_ADDRESS = "0.0.0.0"  # Listen on all available IPs
    PORT = 7777  # The port number to listen on
    start_udp_server(IP_ADDRESS, PORT)
