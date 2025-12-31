#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>

class CameraPublisherNode : public rclcpp::Node {
public:
    CameraPublisherNode();
    void publish_frame();
private:
    rclcpp::TimerBase::SharedPtr read_timer;
    std::unique_ptr<cv::VideoCapture> cap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    int frame_count;
    std::string videoPath = "/home/user-accounts/public/spintop/spinning_and_moving_target.avi";
};