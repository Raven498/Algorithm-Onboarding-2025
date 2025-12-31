#include "../include/armor_detector/armor_detector_node.hpp"

ArmorDetectorNode::ArmorDetectorNode() : Node("armor_detector_node"), frame_count(0)
{
    // Subscribe to the camera publisher topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode subscribed to camera/image_raw");
}

void ArmorDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat frame;
    // *(this->cap) >> frame;

    // if (frame.empty())
    // {
    //     this->cap->release();
    //     cv::destroyAllWindows();
    //     rclcpp::shutdown();
    //     return;
    // }

    try {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<cv::RotatedRect> armors = search(frame, lowerHSV, upperHSV, lowerHSV2, upperHSV2);
    frame_count++;

    if (armors.size() == 2)
    {
        auto p0 = rect_to_point(armors[0]);
        auto p1 = rect_to_point(armors[1]);
        std::cout << frame_count << "," << p0[0] << "," << p0[1] << "," << p1[0] << "," << p1[1] << std::endl;

        draw_rotated_rect(frame, armors[0]);
        draw_rotated_rect(frame, armors[1]);
    }
    else
    {
        std::cout << frame_count << "," << "no armor found" << std::endl;
    }

    if (frame_count % 5 == 0)
    {
        show_frame(frame);
    }
}

void ArmorDetectorNode::show_frame(cv::Mat& frame)
{
    std::vector<uchar> buf;
    cv::resize(frame, frame, cv::Size(640, 480));
    cv::imencode(".jpg", frame, buf, { cv::IMWRITE_JPEG_QUALITY, 20 });
    cv::imshow("Detection Frame", cv::imdecode(buf, cv::IMREAD_COLOR));
    if (cv::waitKey(1) == 27)
    {
        // this->cap->release();
        cv::destroyAllWindows();
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

std::vector<cv::RotatedRect> ArmorDetectorNode::search(cv::Mat& frame, cv::Scalar lowerHSV, cv::Scalar upperHSV, cv::Scalar lowerHSV2, cv::Scalar upperHSV2) {
    cv::Mat mat;

    cv::GaussianBlur(frame, mat, cv::Size(5, 5), 0, 0);

    cv::cvtColor(mat, mat, cv::COLOR_BGR2HSV);

    cv::Mat mat2 = mat.clone();
    cv::inRange(mat, lowerHSV, upperHSV, mat);
    cv::inRange(mat2, lowerHSV2, upperHSV2, mat2);
    cv::bitwise_or(mat, mat2, mat);

    cv::Canny(mat, mat, 100, 200);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mat, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> light_candidates;
    for (int i = 0; i < contours.size(); i++) {
        if (contours[i].size() >= 5) {
            cv::RotatedRect rect = cv::fitEllipse(contours[i]);
            if (is_light_bar(rect)) {
                light_candidates.push_back(rect);
            }
        }
    }

    if (light_candidates.size() >= 2) {
        for (int i = 0; i < light_candidates.size() - 1; i++) {
            cv::RotatedRect bar1 = light_candidates[i];
            for (int j = i + 1; j < light_candidates.size(); j++) {
                cv::RotatedRect bar2 = light_candidates[j];
                if (bar1.center.x < bar2.center.x) {
                    if (is_armor(bar1, bar2)) {
                        return { bar1, bar2 };
                    }
                }
                else {
                    if (is_armor(bar2, bar1)) {
                        return { bar2, bar1 };
                    }
                }
            }
        }
    }

    return {}; // Default return value, no armor found
}

void ArmorDetectorNode::draw_rotated_rect(cv::Mat& frame, cv::RotatedRect& rect)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
}

bool ArmorDetectorNode::is_light_bar(cv::RotatedRect& rect)
{
    if (rect.size.width < LIGHT_BAR_WIDTH_LOWER_LIMIT)
    {
        return false;
    }

    if (rect.size.height < LIGHT_BAR_HEIGHT_LOWER_LIMIT)
    {
        return false;
    }

    if (rect.angle < 180 - LIGHT_BAR_ANGLE_LIMIT && rect.angle > LIGHT_BAR_ANGLE_LIMIT)
    {
        return false;
    }

    if (rect.size.height / rect.size.width < LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT)
    {
        return false;
    }
    return true;
}

bool ArmorDetectorNode::is_armor(cv::RotatedRect& left_rect, cv::RotatedRect& right_rect)
{
    float angle_diff = std::abs(left_rect.angle - right_rect.angle);

    // Light Bar Parallel Check
    if (angle_diff > ARMOR_ANGLE_DIFF_LIMIT && angle_diff < 180 - ARMOR_ANGLE_DIFF_LIMIT)
    {
        return false;
    }

    // Aspect Ratio Ratio Difference Check
    float ar_left = left_rect.size.height / left_rect.size.width;
    float ar_right = right_rect.size.height / right_rect.size.width;
    if (ar_left / ar_right > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT ||
        ar_right / ar_left > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT)
    {
        return false;
    }

    // Light Bar Y Position Ratio Check
    float avg_height = (left_rect.size.height + right_rect.size.height) / 2;
    if (std::abs(left_rect.center.y - right_rect.center.y) / avg_height > ARMOR_Y_DIFF_LIMIT)
    {
        return false;
    }

    // Height Ratio Check
    if (left_rect.size.height / right_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT ||
        right_rect.size.height / left_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT)
    {
        return false;
    }

    // Aspect Ratio Check
    float max_h = std::max(left_rect.size.height, right_rect.size.height);
    float w = cv::norm(left_rect.center - right_rect.center);
    if (w / max_h > ARMOR_ASPECT_RATIO_LIMIT)
    {
        return false;
    }

    return true;
}

std::vector<cv::Point2f> ArmorDetectorNode::rect_to_point(cv::RotatedRect& rect)
{
    float rad = rect.angle < 90 ? rect.angle * M_PI / 180.f : (rect.angle - 180) * M_PI / 180.f;
    float x_offset = rect.size.height * std::sin(rad) / 2.f;
    float y_offset = rect.size.height * std::cos(rad) / 2.f;

    std::vector<cv::Point2f> points;
    points = std::vector<cv::Point2f>();
    points.push_back(cv::Point2f(int(rect.center.x + x_offset), int(rect.center.y - y_offset)));
    points.push_back(cv::Point2f(int(rect.center.x - x_offset), int(rect.center.y + y_offset)));
    return points;
}