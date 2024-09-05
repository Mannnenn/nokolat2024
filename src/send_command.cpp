#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nokolat2024_msg/msg/command.hpp"

class PoseSubscriber : public rclcpp::Node
{
public:
    PoseSubscriber()
        : Node("controller_subscriber")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_command_explicit_topic_name", "/command_explicit");
        this->declare_parameter<std::string>("output_command_topic_name", "/command");

        // パラメータの取得
        std::string input_command_explicit_topic_name;
        this->get_parameter("input_command_explicit_topic_name", input_command_explicit_topic_name);
        std::string output_command_topic_name;
        this->get_parameter("output_command_topic_name", output_command_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        subscription_ = this->create_subscription<nokolat2024_msg::msg::Command>(
            input_command_explicit_topic_name, qos, std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_command_topic_name, 10);
    }

private:
    void pose_callback(const nokolat2024_msg::msg::Command::SharedPtr msg)
    {
        geometry_msgs::msg::Twist command_send_msg;
        command_send_msg.linear.x = msg->throttle;
        command_send_msg.linear.y = msg->elevator;
        command_send_msg.linear.z = msg->rudder;
        command_send_msg.angular.x = msg->aileron_r;
        command_send_msg.angular.y = msg->dropping_device;
        command_send_msg.angular.z = msg->aileron_l;

        publisher_->publish(command_send_msg);

        // RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f, %f, %f, %f, %f, %d]", msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, static_cast<int16_t>(msg->orientation.w));
    }

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}