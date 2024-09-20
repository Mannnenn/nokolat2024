#include "rclcpp/rclcpp.hpp"
#include "nokolat2024_msg/msg/command.hpp"

using namespace std::chrono_literals;

class NeutralPositionCalculator : public rclcpp::Node
{
public:
    NeutralPositionCalculator()
        : Node("neutral_position_calculator"), count_(0)
    {
        this->declare_parameter<std::string>("input_command_explicit_topic_name", "/command_explicit");
        this->declare_parameter<std::string>("output_neutral_position_topic_name", "/neutral");

        std::string input_command_explicit_topic_name;
        this->get_parameter("input_command_explicit_topic_name", input_command_explicit_topic_name);
        std::string output_neutral_position_topic_name;
        this->get_parameter("output_neutral_position_topic_name", output_neutral_position_topic_name);

        subscription_ = this->create_subscription<nokolat2024_msg::msg::Command>(
            input_command_explicit_topic_name, 10, std::bind(&NeutralPositionCalculator::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<nokolat2024_msg::msg::Command>(output_neutral_position_topic_name, 10);
    }

private:
    void topic_callback(const nokolat2024_msg::msg::Command::SharedPtr msg)
    {
        if (first_callback)
        {
            start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Processing neutral position...");
            first_callback = false;
        }
        rclcpp::Time current_time = this->now();
        rclcpp::Duration duration = current_time - start_time_;

        if (duration.seconds() < 1.0)
        {
            sum_throttle_ += msg->throttle;
            sum_elevator_ += msg->elevator;
            sum_rudder_ += msg->rudder;
            sum_aileron_r_ += msg->aileron_r;
            sum_aileron_l_ += msg->aileron_l;
            sum_dropping_device_ += msg->dropping_device;
            count_++;
        }
        else
        {
            nokolat2024_msg::msg::Command message;
            message.throttle = static_cast<float>(sum_throttle_ / count_);
            message.elevator = static_cast<float>(sum_elevator_ / count_);
            message.rudder = static_cast<float>(sum_rudder_ / count_);
            message.aileron_r = static_cast<float>(sum_aileron_r_ / count_);
            message.aileron_l = static_cast<float>(sum_aileron_l_ / count_);
            message.dropping_device = static_cast<float>(sum_dropping_device_ / count_);

            publisher_->publish(message);
        }
    }

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr subscription_;
    rclcpp::Publisher<nokolat2024_msg::msg::Command>::SharedPtr publisher_;

    double sum_throttle_ = 0.0;
    double sum_elevator_ = 0.0;
    double sum_rudder_ = 0.0;
    double sum_aileron_r_ = 0.0;
    double sum_aileron_l_ = 0.0;
    double sum_dropping_device_ = 0.0;
    int count_;

    rclcpp::Time start_time_;
    bool first_callback = true;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NeutralPositionCalculator>());
    return 0;
}