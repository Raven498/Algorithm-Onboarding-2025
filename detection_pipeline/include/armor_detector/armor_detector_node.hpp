#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// Constants
#define HUE_RANGE_LIMIT 30.0
#define SATURATION_LOWER_LIMIT 100.0
#define VALUE_LOWER_LIMIT 150.0
#define LIGHT_BAR_ANGLE_LIMIT 30.0
#define LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT 2.0
#define LIGHT_BAR_WIDTH_LOWER_LIMIT 2.0
#define LIGHT_BAR_HEIGHT_LOWER_LIMIT 5.0
#define ARMOR_ANGLE_DIFF_LIMIT 5.0
#define ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT 5.0
#define ARMOR_Y_DIFF_LIMIT 1.5
#define ARMOR_HEIGHT_RATIO_LIMIT 1.5
#define ARMOR_ASPECT_RATIO_LIMIT 2.5

// Define the HSV color range for segmentation 
cv::Scalar lowerHSV(0, 120, 70);   // Lower bound of HSV
cv::Scalar upperHSV(10, 255, 255); // Upper bound of HSV
cv::Scalar lowerHSV2(170, 120, 70);   // Lower bound of HSV
cv::Scalar upperHSV2(179, 255, 255); // Upper bound of HSV

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode();
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    // std::unique_ptr<cv::VideoCapture> cap;

    int frame_count;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void show_frame(cv::Mat& frame);

    std::vector<cv::RotatedRect> search(cv::Mat& frame, cv::Scalar lowerHSV, cv::Scalar upperHSV, cv::Scalar lowerHSV2, cv::Scalar upperHSV2);
    bool is_light_bar(cv::RotatedRect& rect);
    bool is_armor(cv::RotatedRect& left_rect, cv::RotatedRect& right_rect);
    void draw_rotated_rect(cv::Mat& frame, cv::RotatedRect& rect);
    std::vector<cv::Point2f> rect_to_point(cv::RotatedRect& rect);
};