#include "nokolat2024/main_control.hpp"

class UIcommand : public rclcpp::Node
{
public:
    UIcommand()
        : Node("controller_subscriber")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_command_topic_name", "/command_send");
        this->declare_parameter<std::string>("input_command_explicit_topic_name", "/command_receive");
        this->declare_parameter<std::string>("input_mode_topic_name", "/mode");

        // パラメータの取得
        std::string input_command_topic_name;
        this->get_parameter("input_command_topic_name", input_command_topic_name);
        std::string input_command_explicit_topic_name;
        this->get_parameter("input_command_explicit_topic_name", input_command_explicit_topic_name);
        std::string input_mode_topic_name;
        this->get_parameter("input_mode_topic_name", input_mode_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        subscription_manual_ = this->create_subscription<nokolat2024_msg::msg::Command>(
            input_command_topic_name, qos, std::bind(&UIcommand::manual_callback, this, std::placeholders::_1));

        subscription_auto_ = this->create_subscription<nokolat2024_msg::msg::Command>(
            input_command_explicit_topic_name, qos, std::bind(&UIcommand::auto_callback, this, std::placeholders::_1));

        subscription_mode_ = this->create_subscription<std_msgs::msg::String>(
            input_mode_topic_name, qos, std::bind(&UIcommand::mode_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        throttle_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/throttle_command", qos_settings);
        elevator_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/elevator_command", qos_settings);
        aileron_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/aileron_command", qos_settings);
        rudder_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/rudder_command", qos_settings);
    }

private:
    void manual_callback(const nokolat2024_msg::msg::Command::SharedPtr msg)
    {
        if (mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::MANUAL))
        {
            throttle_ = msg->throttle;
            elevator_ = msg->elevator;
            aileron_ = msg->aileron_l;
            rudder_ = msg->rudder;

            pub_ui_data();
        }

        // RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f, %f, %f, %f, %f, %d]", msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, static_cast<int16_t>(msg->orientation.w));
    }

    void auto_callback(const nokolat2024_msg::msg::Command::SharedPtr msg)
    {
        if (mode_ != nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::MANUAL))
        {
            throttle_ = msg->throttle;
            elevator_ = msg->elevator;
            aileron_ = msg->aileron_l;
            rudder_ = msg->rudder;

            pub_ui_data();
        }
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        mode_ = msg->data;
    }

    void pub_ui_data()
    {
        std_msgs::msg::Float32 throttle_command_msg;
        throttle_command_msg.data = throttle_;
        throttle_command_publisher_->publish(throttle_command_msg);

        std_msgs::msg::Float32 elevator_command_msg;
        elevator_command_msg.data = elevator_;
        elevator_command_publisher_->publish(elevator_command_msg);

        std_msgs::msg::Float32 aileron_command_msg;
        aileron_command_msg.data = aileron_;
        aileron_command_publisher_->publish(aileron_command_msg);

        std_msgs::msg::Float32 rudder_command_msg;
        rudder_command_msg.data = rudder_;
        rudder_command_publisher_->publish(rudder_command_msg);
    }

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr subscription_manual_;
    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr subscription_auto_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_mode_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aileron_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_command_publisher_;

    double throttle_;
    double elevator_;
    double aileron_;
    double rudder_;

    std::string mode_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UIcommand>());
    rclcpp::shutdown();
    return 0;
}