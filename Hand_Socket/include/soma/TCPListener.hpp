#ifndef SOMA_TCPLISTENER_HPP_
#define SOMA_TCPLISTENER_HPP_

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

#include <string>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <memory.h>
#include <chrono>

class TCPListener
{
private:
    // Socket variables 
    std::string ip_address;
    int port;
    int server_fd;
    struct sockaddr_in address;
    int new_socket;
    char buffer[1024];
    ros::Time start_time;
    float start_utc_second;
    std::string device;

    // ROS variables
    ros::NodeHandle nh;
    std::unique_ptr<tf::TransformBroadcaster> br;
    std::unique_ptr<tf::StampedTransform> transform;

    ros::Publisher left_humerus_pub;
    ros::Publisher left_ulna_pub;
    ros::Publisher left_wrist_pub;

    ros::Publisher right_humerus_pub;
    ros::Publisher right_ulna_pub;
    ros::Publisher right_wrist_pub;

public:
    TCPListener();
    int ListenPort();
    void ListenSendTransform();
    ~TCPListener();
};

#endif