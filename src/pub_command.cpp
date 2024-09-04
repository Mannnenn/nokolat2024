#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToInt16ArrayPublisher : public rclcpp::Node
{
public:
    JoyToInt16ArrayPublisher()
        : Node("command_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("micro_ros_arduino_twist_subscriber", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyToInt16ArrayPublisher::joyCallback, this, std::placeholders::_1));
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto message = geometry_msgs::msg::Twist();

        // Joyメッセージのaxes配列の最初の5つの要素を使用してTwistメッセージを作成
        message.angular.x = static_cast<float>(msg->axes.size() > 0 ? ((msg->axes[0] + 1) / 2) * 1024 : 0);
        message.angular.y = static_cast<float>(msg->axes.size() > 1 ? ((msg->axes[1] + 1) / 2) * 1024 : 0);
        message.angular.z = static_cast<float>(msg->axes.size() > 2 ? ((msg->axes[2] + 1) / 2) * 1024 : 0);
        float throttle = msg->axes.size() > 3 ? msg->axes[3] : 0;
        // 中立位置でのスロットル値を0にする
        if (throttle < 0)
        {
            throttle = 0;
        }
        message.linear.x = static_cast<float>(throttle * 1024);
        message.linear.y = static_cast<float>(msg->axes.size() > 4 ? ((msg->axes[4] + 1) / 2) * 1024 : 0);
        message.linear.z = static_cast<float>(msg->axes.size() > 5 ? ((msg->axes[5] + 1) / 2) * 1024 : 0);

        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToInt16ArrayPublisher>());
    rclcpp::shutdown();
    return 0;
}