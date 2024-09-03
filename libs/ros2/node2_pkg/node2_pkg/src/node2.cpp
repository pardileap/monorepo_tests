#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Node2 : public rclcpp::Node
{
public:
    Node2() : Node("node2")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("node2_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&Node2::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from node2";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Node2>());
    rclcpp::shutdown();
    return 0;
}
