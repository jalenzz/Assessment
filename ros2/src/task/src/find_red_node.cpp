#include <memory>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/u_int8.hpp"
#include "my_interfaces/msg/my_mat.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
    : Node("image_subscriber") , red_count(0) {
        subscription_ = this->create_subscription<my_interfaces::msg::MyMat>(
            "raw_image", 10, std::bind(&ImageSubscriber::topic_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::UInt8>("red_count", 10);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&ImageSubscriber::timer_callback, this));
    }

private:
    void topic_callback(const my_interfaces::msg::MyMat &msg) {
        auto cv_ptr = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::TYPE_8UC3);
        auto cv_image = cv_ptr->image;

        double fps = 0.0;
        cv::Mat hsv_image, red_mask;
        RCLCPP_INFO(this->get_logger(), "get image '%s'", msg.time_stamp.c_str());

        cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);

        // 提取红色区域
        auto lower = cv::Scalar(0, 100, 100);  // 红色的 HSV 下界
        auto upper = cv::Scalar(10, 255, 255); // 红色的 HSV 上界
        inRange(hsv_image, lower, upper, red_mask);

        // 闭运算连接红色区域，开运算去噪点
        auto kernel_close = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        auto kernel_open = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel_close);
        morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel_open);

        std::vector<std::vector<cv::Point>> contours;
        findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for(auto contour : contours) {
            auto rect = boundingRect(contour);
            rectangle(cv_image, rect, cv::Scalar(0, 0, 255), 2);
        }

        red_count = contours.size();
        RCLCPP_INFO(this->get_logger(), "find red'%d'", red_count);
        cv::putText(cv_image, "FPS: " + std::to_string(fps) + "\n" + msg.time_stamp,
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::imshow("Result", cv_image);
        cv::imshow("RedMask", red_mask);
        cv::waitKey(10);
    }
    void timer_callback() {
        auto message = std_msgs::msg::UInt8();
        message.data = (char)red_count;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
        publisher_->publish(message);
    }
    uint8_t red_count;      // 红色物体数量
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<my_interfaces::msg::MyMat>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
