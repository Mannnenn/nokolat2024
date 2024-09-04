#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nokolat2024_msg/msg/command.hpp"
#include <unordered_map>
#include "std_msgs/msg/string.hpp"

class PoseSubscriber : public rclcpp::Node
{
public:
    PoseSubscriber()
        : Node("controller_subscriber")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_propo_topic_name", "/command");
        this->declare_parameter<std::string>("output_command_explicit_topic_name", "/command_explicit");
        this->declare_parameter<std::string>("output_mode_topic_name", "/mode");

        // パラメータの取得
        std::string input_propo_topic_name;
        this->get_parameter("input_propo_topic_name", input_propo_topic_name);
        std::string output_command_explicit_topic_name;
        this->get_parameter("output_command_explicit_topic_name", output_command_explicit_topic_name);
        std::string output_mode_topic_name;
        this->get_parameter("output_mode_topic_name", output_mode_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            input_propo_topic_name, qos, std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1));

        publisher_custom_ = this->create_publisher<nokolat2024_msg::msg::Command>(output_command_explicit_topic_name, 10);
        publisher_mode_ = this->create_publisher<std_msgs::msg::String>(output_mode_topic_name, 10);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        nokolat2024_msg::msg::Command command_receive_msg;
        command_receive_msg.throttle = msg->position.x;
        command_receive_msg.elevator = msg->position.y;
        command_receive_msg.rudder = msg->position.z;
        command_receive_msg.aileron_r = msg->orientation.x;
        command_receive_msg.aileron_l = msg->orientation.y;
        command_receive_msg.dropping_device = msg->orientation.z;

        command_receive_msg.header.stamp = this->now();
        command_receive_msg.header.frame_id = "controller";

        auto mode_msg = std_msgs::msg::String();

        auto it = control_mode_map.find(static_cast<int16_t>(msg->orientation.w));
        if (it != control_mode_map.end())
        {
            mode_msg.data = it->second;
        }

        publisher_custom_->publish(command_receive_msg);
        publisher_mode_->publish(mode_msg);

        // RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f, %f, %f, %f, %f, %d]", msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, static_cast<int16_t>(msg->orientation.w));
    }

    // Define control mode
    enum CONTROL_MODE
    {
        MANUAL = 0,
        AUTO_TURNING = 1,
        AUTO_RISE_TURNING = 2,
        AUTO_LANDING = 3,
        AUTO_EIGHT = 4,
    };

    const std::unordered_map<int16_t, std::string> control_mode_map = {
        {CONTROL_MODE::MANUAL, "MANUAL"},
        {CONTROL_MODE::AUTO_TURNING, "AUTO_TURNING"},
        {CONTROL_MODE::AUTO_RISE_TURNING, "AUTO_RISE_TURNING"},
        {CONTROL_MODE::AUTO_LANDING, "AUTO_LANDING"},
        {CONTROL_MODE::AUTO_EIGHT, "AUTO_EIGHT"}};

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<nokolat2024_msg::msg::Command>::SharedPtr publisher_custom_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_mode_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}