#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "my_interfaces/msg/my_mat.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node{
public:
    ImagePublisher()
    : Node("ImagePublisher"), count_(0) {
        capture.open(0);
        publisher_ = this->create_publisher<my_interfaces::msg::MyMat>("raw_image", 10);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        time_t now = time(0);
        capture >> cv_image;
        my_interfaces::msg::MyMat message;

        message.time_stamp = std::to_string(now);
        std_msgs::msg::Header header;
        message.image = *cv_bridge::CvImage(header, "bgr8", cv_image).toImageMsg();
        // message.image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg(); 
        ++count_;

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.time_stamp.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_interfaces::msg::MyMat>::SharedPtr publisher_;
    size_t count_;
    cv::VideoCapture capture;
    cv::Mat cv_image;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
