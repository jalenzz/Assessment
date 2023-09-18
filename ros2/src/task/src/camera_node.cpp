#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "my_interfaces/msg/my_mat.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("ImagePublisher") {
        capture.open(0);
        publisher_ = this->create_publisher<my_interfaces::msg::MyMat>("raw_image", 10);
        timer_ = this->create_wall_timer(
            1ms, std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto now = rclcpp::Clock().now().seconds();
        capture >> cv_image;
        my_interfaces::msg::MyMat message;

        message.time.data = now;
        std_msgs::msg::Header header;
        message.image = *cv_bridge::CvImage(header, "bgr8", cv_image).toImageMsg();

        RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", message.time.data);
        publisher_->publish(message);
    }
    cv::Mat cv_image;
    cv::VideoCapture capture;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_interfaces::msg::MyMat>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
