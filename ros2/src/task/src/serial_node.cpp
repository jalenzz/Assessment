#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;

class redCountSubscriber : public rclcpp::Node {
public:
    redCountSubscriber() : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
            "red_count", 10, std::bind(&redCountSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::UInt8 &msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
    }
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<redCountSubscriber>());
    rclcpp::shutdown();
    return 0;
}
