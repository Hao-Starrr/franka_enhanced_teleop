#include "soma/TCPListener.hpp"

std::map<int, std::string> bone_map = {
    {0, "RightRistRoot"},
    {1, "RightForearmStub"},
    {2, "RightThumb0"},
    {3, "RightThumb1"},
    {4, "RightThumb2"},
    {5, "RightThumb3"},
    {6, "RightIndex1"},
    {7, "RightIndex2"},
    {8, "RightIndex3"},
    {9, "RightMiddle1"},
    {10, "RightMiddle2"},
    {11, "RightMiddle3"},
    {12, "RightRing1"},
    {13, "RightRing2"},
    {14, "RightRing3"},
    {15, "RightPinky0"},
    {16, "RightPinky1"},
    {17, "RightPinky2"},
    {18, "RightPinky3"},
    {19, "RightThumbTip"},
    {20, "RightIndexTip"},
    {21, "RightMiddleTip"},
    {22, "RightRingTip"},
    {23, "RightPinkyTip"},
    {24, "RightEnd"},
    {0 + 25, "LeftRistRoot"},
    {1 + 25, "LeftForearmStub"},
    {2 + 25, "LeftThumb0"},
    {3 + 25, "LeftThumb1"},
    {4 + 25, "LeftThumb2"},
    {5 + 25, "LeftThumb3"},
    {6 + 25, "LeftIndex1"},
    {7 + 25, "LeftIndex2"},
    {8 + 25, "LeftIndex3"},
    {9 + 25, "LeftMiddle1"},
    {10 + 25, "LeftMiddle2"},
    {11 + 25, "LeftMiddle3"},
    {12 + 25, "LeftRing1"},
    {13 + 25, "LeftRing2"},
    {14 + 25, "LeftRing3"},
    {15 + 25, "LeftPinky0"},
    {16 + 25, "LeftPinky1"},
    {17 + 25, "LeftPinky2"},
    {18 + 25, "LeftPinky3"},
    {19 + 25, "LeftThumbTip"},
    {20 + 25, "LeftIndexTip"},
    {21 + 25, "LeftMiddleTip"},
    {22 + 25, "LeftRingTip"},
    {23 + 25, "LeftPinkyTip"},
    {24 + 25, "LeftEnd"}  
};

std::map<int, std::string> ultraleap_map = {
    {0, "RightWrist"},
    {1, "RightElbow"},
    {2, "RightThumbMETA"},
    {3, "RightThumbPROX"},
    {4, "RightThumbINTE"},
    {5, "RightThumbDIST"},
    {6, "RightIndexMETA"},
    {7, "RightIndexPROX"},
    {8, "RightIndexINTE"},
    {9, "RightIndexDIST"},
    {10, "RightMiddleMETA"},
    {11, "RightMiddlePROX"},
    {12, "RightMiddleINTE"},
    {13, "RightMiddleDIST"},
    {14, "RightRingMETA"},
    {15, "RightRingPROX"},
    {16, "RightRingINTE"},
    {17, "RightRingDIST"},
    {18, "RightPinkyMETA"},
    {19, "RightPinkyPROX"},
    {20, "RightPinkyINTE"},
    {21, "RightPinkyDIST"},
    {0 + 25, "LeftWrist"},
    {1 + 25, "LeftElbow"},
    {2 + 25, "LeftThumbMETA"},
    {3 + 25, "LeftThumbPROX"},
    {4 + 25, "LeftThumbINTE"},
    {5 + 25, "LeftThumbDIST"},
    {6 + 25, "LeftIndexMETA"},
    {7 + 25, "LeftIndexPROX"},
    {8 + 25, "LeftIndexINTE"},
    {9 + 25, "LeftIndexDIST"},
    {10 + 25, "LeftMiddleMETA"},
    {11 + 25, "LeftMiddlePROX"},
    {12 + 25, "LeftMiddleINTE"},
    {13 + 25, "LeftMiddleDIST"},
    {14 + 25, "LeftRingMETA"},
    {15 + 25, "LeftRingPROX"},
    {16 + 25, "LeftRingINTE"},
    {17 + 25, "LeftRingDIST"},
    {18 + 25, "LeftPinkyMETA"},
    {19 + 25, "LeftPinkyPROX"},
    {20 + 25, "LeftPinkyINTE"},
    {21 + 25, "LeftPinkyDIST"}
};

TCPListener::TCPListener()
    :nh("~")
{
    // Get Ros parameters
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<int>("port", port, 8080);
    nh.param<std::string>("device", device, "quest");

    start_time = ros::Time::now();

    // Advertise Topic
    left_humerus_pub = nh.advertise<geometry_msgs::TransformStamped>("left_humerus", 1000);
    left_ulna_pub = nh.advertise<geometry_msgs::TransformStamped>("left_ulna", 1000);
    left_wrist_pub = nh.advertise<geometry_msgs::TransformStamped>("left_wrist", 1000);

    right_humerus_pub = nh.advertise<geometry_msgs::TransformStamped>("right_humerus", 1000);
    right_ulna_pub = nh.advertise<geometry_msgs::TransformStamped>("right_ulna", 1000);
    right_wrist_pub = nh.advertise<geometry_msgs::TransformStamped>("right_wrist", 1000);

    // get current UTC time
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* utc_time = std::gmtime(&now_c);

    auto millisecond = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    start_utc_second = utc_time->tm_hour * 3600 + utc_time->tm_min * 60 + utc_time->tm_sec;
    start_utc_second += static_cast<float>(millisecond.count()) / 1000.0;

    ROS_INFO_STREAM("Listen to " << ip_address << " Port: " << port);

    // listen to the ip_address and port
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0)
    {
        ROS_ERROR("Failed to create socket");
        exit(EXIT_FAILURE);
    }

    int addrlen = sizeof(address);
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(ip_address.c_str());
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        ROS_ERROR("Failed to bind socket");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0)
    {
        ROS_ERROR("Failed to listen to socket");
        exit(EXIT_FAILURE);
    }

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);
    if (new_socket < 0)
    {
        ROS_ERROR("Failed to accept socket");
        exit(EXIT_FAILURE);
    }

    // allocate buffer

    // initialize ROS variables
    br = std::make_unique<tf::TransformBroadcaster>();
}

/*
    * Listen to the port and return the number of bytes read
*/
int TCPListener::ListenPort()
{
    int valread = read(new_socket, buffer, 36 * 8);

    if (valread < 0)
    {
        ROS_ERROR("Failed to read from socket");
        exit(EXIT_FAILURE);
    }

    // Calculate remaining bytes for a complete message
    int num_floats = valread / 36;
    int res = valread - num_floats * 36;
    if (res != 0){
        int expected_read = (num_floats + 1) * 36;
        while(valread < expected_read){
            valread += read(new_socket, buffer + valread, expected_read - valread);
        }
    }
    
    return valread;
}

/*
    Listen from socket and send transform to ros tf
*/
void TCPListener::ListenSendTransform()
{
    // listen to the port
    int valread = ListenPort();

    if(device == std::string("quest")){
        // The encode is time, bone_id, x, y, z, qx, qy, qz, qw; 
        // each is encoded as a float32

        // parse the buffer
        float* buffer_float = reinterpret_cast<float*>(buffer);
        for(int i = 0; i < valread / 36; i++)
        {
            float msg_utc_second = buffer_float[0];
            double msg_ros_second = start_time.toSec() + msg_utc_second - start_utc_second;
            ROS_WARN_STREAM("msg_utc_second: " << msg_utc_second << " msg_ros_second: " << msg_ros_second);
            
            ros::Time msg_ros_time;
            msg_ros_time.fromSec(msg_ros_second);

            // create transform
            transform = std::make_unique<tf::StampedTransform>(
                tf::StampedTransform(
                    tf::Transform(
                        tf::Quaternion(buffer_float[5], buffer_float[6], buffer_float[7], buffer_float[8]), 
                        tf::Vector3(buffer_float[2], buffer_float[3], buffer_float[4])), 
                    ros::Time::now(), 
                    "world", 
                    bone_map[static_cast<int>(buffer_float[1])]
                )
            );

            // // send transform
            br->sendTransform(*transform);

            // ROS_WARN_STREAM(" Hand ID: " << static_cast<int>(buffer_float[1]) << " x: " << buffer_float[2] << " y: " << buffer_float[3] << " z: " 
            //     << buffer_float[4] << " qx: " << buffer_float[5] << " qy: " 
            //     << buffer_float[6] << " qz: " << buffer_float[7] << " qw: " << buffer_float[8]);

            // increment buffer
            buffer_float += 9;
        }
    } else if (device == std::string("ultraleap")){

        // parse the buffer
        float* buffer_float = reinterpret_cast<float*>(buffer);
        for(int i = 0; i < valread / 36; i++)
        {
            float msg_utc_second = buffer_float[0];
            double msg_ros_second = start_time.toSec() + msg_utc_second - start_utc_second;
            // ROS_WARN_STREAM("msg_utc_second: " << msg_utc_second << " msg_ros_second: " << msg_ros_second);
            
            // ros::Time msg_ros_time;
            // msg_ros_time.fromSec(msg_ros_second);

            geometry_msgs::TransformStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "world";

            pose_msg.transform.translation.x = buffer_float[2];
            pose_msg.transform.translation.y = buffer_float[3];
            pose_msg.transform.translation.z = buffer_float[4];

            pose_msg.transform.rotation.x = buffer_float[5];
            pose_msg.transform.rotation.y = buffer_float[6];
            pose_msg.transform.rotation.z = buffer_float[7];
            pose_msg.transform.rotation.w = buffer_float[8];

            // publish to topic
            if (static_cast<int>(buffer_float[1]) == 25){
                pose_msg.child_frame_id = "right_ulna";
                left_ulna_pub.publish(pose_msg);
            } else if (static_cast<int>(buffer_float[1]) == 26){
                pose_msg.child_frame_id = "right_humerus";
                left_humerus_pub.publish(pose_msg);
            } else if (static_cast<int>(buffer_float[1]) == 35){
                pose_msg.child_frame_id = "right_wrist";
                left_wrist_pub.publish(pose_msg);
            }  if (static_cast<int>(buffer_float[1]) == 0){
                pose_msg.child_frame_id = "left_ulna";
                right_ulna_pub.publish(pose_msg);
            } else if (static_cast<int>(buffer_float[1]) == 1){
                pose_msg.child_frame_id = "left_humerus";
                right_humerus_pub.publish(pose_msg);
            } else if (static_cast<int>(buffer_float[1]) == 10){
                pose_msg.child_frame_id = "left_wrist";
                right_wrist_pub.publish(pose_msg);
            }  else if (static_cast<int>(buffer_float[1]) == 22) {
                // Right Hand Pinch State
                float PinchDistance = buffer_float[5];
                float PinchStrength = buffer_float[6];

                if(PinchStrength > 0.8){
                    ROS_WARN(" Right Hand Pinch Detected !");
                }
            }

            // increment buffer
            buffer_float += 9;
        }

    } else {
        ROS_ERROR_STREAM(" Invalid device : " << device);
    }
}

TCPListener::~TCPListener()
{
    close(new_socket);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "socket_node");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    TCPListener listener;
    while(ros::ok())
    {
        listener.ListenSendTransform();
        ros::spinOnce();
    }
}