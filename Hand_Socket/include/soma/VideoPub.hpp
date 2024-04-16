#ifndef SOMA_TCPLISTENER_HPP_
#define SOMA_TCPLISTENER_HPP_

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

#include <string>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <memory.h>
#include <chrono>

class ImageSubscriber
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
    std::string image_topic;

    // ROS variables
    ros::NodeHandle nh;
    ros::Subscriber image_sub;

public:
    ImageSubscriber();
    void ImageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif