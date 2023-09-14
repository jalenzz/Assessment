#include <memory>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/string.hpp"
#include "my_interfaces/msg/my_mat.hpp"

using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
    : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<my_interfaces::msg::MyMat>(
            "topic", 10, std::bind(&ImageSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const my_interfaces::msg::MyMat &msg) const {
        auto cv_ptr = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::TYPE_8UC3);
        cv::Mat img__ = cv_ptr->image;
        cv::imshow("test", img__);
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.time_stamp.c_str());
    }
    rclcpp::Subscription<my_interfaces::msg::MyMat>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
