#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"

class TimerNode : public rclcpp::Node
{
public:
    TimerNode() : Node("timer_node"), start_time_(0), timer_started_(false)
    {
        start_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "start", 10, std::bind(&TimerNode::start_callback, this, std::placeholders::_1));

        timer_publisher_ = this->create_publisher<std_msgs::msg::Float32>("timer", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TimerNode::timer_callback, this));
    }

private:
    void start_callback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        auto hoge = msg;
        start_time_ = this->now();
        timer_started_ = true;
    }

    void timer_callback()
    {
        if (timer_started_)
        {
            auto message = std_msgs::msg::Float32();
            float elapsed_time = (this->now() - start_time_).seconds();
            int minutes = static_cast<int>(elapsed_time) / 60;
            int seconds = static_cast<int>(elapsed_time - (minutes * 60));
            message.data = minutes + (static_cast<float>(seconds) / 100);
            timer_publisher_->publish(message);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timer_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    bool timer_started_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimerNode>());
    rclcpp::shutdown();
    return 0;
}