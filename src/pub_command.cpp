#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

class Int16ArrayPublisher : public rclcpp::Node
{
public:
    Int16ArrayPublisher()
        : Node("int16_array_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("int16_array_topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Int16ArrayPublisher::publishArray, this));
    }

private:
    void publishArray()
    {
        auto message = std_msgs::msg::Int16MultiArray();
        message.data.size = 5;               // 配列の要素数を設定
        message.data.data = {1, 2, 3, 4, 5}; // 配列の要素を設定
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Int16ArrayPublisher>());
    rclcpp::shutdown();
    return 0;
}