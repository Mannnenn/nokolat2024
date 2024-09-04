#include <rclcpp/rclcpp.hpp>
#include "nokolat2024_msg/msg/rpy.hpp"
#include "nokolat2024_msg/msg/command.hpp"
#include "std_msgs/msg/string.hpp"
#include <unordered_map>
#include <string>

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
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

        string_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "string_topic", 10, std::bind(&MyNode::string_callback, this, std::placeholders::_1));

        rpy_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Rpy>(
            "rpy_topic", 10, std::bind(&MyNode::rpy_callback, this, std::placeholders::_1));

        altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "altitude_stamped_topic", 10, std::bind(&MyNode::altitude_callback, this, std::placeholders::_1));

        command_publisher_ = this->create_publisher<nokolat2024_msg::msg::Command>("command_topic", 10);
    }

private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received string: '%s'", msg->data.c_str());
        control_mode_ = msg->data;
    }

    void rpy_callback(const nokolat2024_msg::msg::Rpy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received RPY: roll=%f, pitch=%f, yaw=%f", msg->roll, msg->pitch, msg->yaw);
        // ここでRPYメッセージを処理し、Commandメッセージをパブリッシュ
        auto command_msg = nokolat2024_msg::msg::Command();
        // Commandメッセージのフィールドを設定
        command_publisher_->publish(command_msg);
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

    // マップを初期化
    const std::unordered_map<int16_t, std::string> control_mode_map = {
        {CONTROL_MODE::MANUAL, "MANUAL"},
        {CONTROL_MODE::AUTO_TURNING, "AUTO_TURNING"},
        {CONTROL_MODE::AUTO_RISE_TURNING, "AUTO_RISE_TURNING"},
        {CONTROL_MODE::AUTO_LANDING, "AUTO_LANDING"},
        {CONTROL_MODE::AUTO_EIGHT, "AUTO_EIGHT"}};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Publisher<nokolat2024_msg::msg::Command>::SharedPtr command_publisher_;

    string control_mode_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}