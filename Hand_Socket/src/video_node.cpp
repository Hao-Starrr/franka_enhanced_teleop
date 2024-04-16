#include "soma/VideoPub.hpp"

ImageSubscriber::ImageSubscriber()
    :nh("~")
{
    nh.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh.param<std::string>("image_topic", image_topic, "/image");
    nh.param<int>("img_port", port, 8009);

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

    image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 1, &ImageSubscriber::ImageCallback, this);
}

void ImageSubscriber::ImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // convert to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Resize to 256 x 256
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(256, 256));

    // convert to jpeg
    std::vector<uchar> buf;
    std::vector<int> param = std::vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 80;
    cv::imencode(".jpg", cv_ptr->image, buf, param);

    // send image size
    int buf_size = buf.size();
    send(new_socket, (char *)&buf_size, sizeof(int), 0);

    // send the image
    send(new_socket, buf.data(), buf.size(), 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_pub");
    ImageSubscriber image_subscriber;
    ros::spin();
    return 0;
}