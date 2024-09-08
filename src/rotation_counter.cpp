#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <cmath>
#include <deque>

#include "nokolat2024_msg/msg/rpy.hpp"
#include "nokolat2024_msg/msg/command.hpp"

#include "std_msgs/msg/float32.hpp"

class RotationCounterNode : public rclcpp::Node
{
public:
    RotationCounterNode()
        : Node("rotation_counter_node")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_angular_topic_name", "/rpy");
        this->declare_parameter<std::string>("input_angular_velocity_topic_name", "/angular_velocity");
        this->declare_parameter<std::string>("input_counter_reset_topic_name", "/counter_reset");
        this->declare_parameter<std::string>("output_rotation_counter_topic_name", "/rotation_counter");

        // パラメータの取得
        std::string input_angular_topic_name;
        this->get_parameter("input_angular_topic_name", input_angular_topic_name);
        std::string input_angular_velocity_topic_name;
        this->get_parameter("input_angular_velocity_topic_name", input_angular_velocity_topic_name);
        std::string input_counter_reset_topic_name;
        this->get_parameter("input_counter_reset_topic_name", input_counter_reset_topic_name);
        std::string output_rotation_counter_topic_name;
        this->get_parameter("output_rotation_counter_topic_name", output_rotation_counter_topic_name);

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        rpy_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Rpy>(input_angular_topic_name, qos, std::bind(&RotationCounterNode::rpy_callback, this, std::placeholders::_1));
        angular_velocity_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Rpy>(input_angular_velocity_topic_name, qos, std::bind(&RotationCounterNode::angular_velocity_callback, this, std::placeholders::_1));
        counter_reset_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(input_counter_reset_topic_name, qos, std::bind(&RotationCounterNode::counter_reset_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "rotation_counter_node has been started.");

        rclcpp::QoS qos_settings(10);
        qos_settings.reliable();

        rotation_counter_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_rotation_counter_topic_name, qos_settings);

        epsilon_ = 0.2;
        over_2pi_ = 0;
        yaw_offset_ = 0;
        last_time_ = this->now();
    }

private:
    void rpy_callback(const nokolat2024_msg::msg::Rpy::SharedPtr msg)
    {
        if (is_first_counter_)
        {
            start_yaw_ = msg->yaw;
            previous_yaw_ = msg->yaw;
            over_2pi_ = 0;
            yaw_offset_ = 0;
            is_first_counter_ = false;

            RCLCPP_INFO(this->get_logger(), "Start counting");
        }

        previous_yaw_ = msg->yaw;

        if (rotation_direction_ == 1 && previous_yaw_ > M_PI - epsilon_)
        {
            over_2pi_ = 1;
        }
        else if (rotation_direction_ == -1 && previous_yaw_ < -M_PI + epsilon_)
        {
            over_2pi_ = -1;
        }

        if (over_2pi_ == 1 && previous_yaw_ > -M_PI)
        {
            yaw_offset_ = 2 * M_PI;
        }
        else if (over_2pi_ == -1 && previous_yaw_ < M_PI)
        {
            yaw_offset_ = -2 * M_PI;
        }
        else
        {
            yaw_offset_ = 0;
        }

        double yaw = previous_yaw_ + yaw_offset_;

        // RCLCPP_INFO(this->get_logger(), "Over boundary value %d", over_2pi_);
        // RCLCPP_INFO(this->get_logger(), "Rotation %f", rotation_direction_);
        // RCLCPP_INFO(this->get_logger(), "before offset %f", previous_yaw_);
        // RCLCPP_INFO(this->get_logger(), "after offset %f", yaw);
        // RCLCPP_INFO(this->get_logger(), "start yaw %f", start_yaw_);
        // RCLCPP_INFO(this->get_logger(), "rotation counter %f", rotation_counter_);

        rclcpp::Time current_time = this->now();
        rclcpp::Duration diff = current_time - last_time_;

        double epsilon_judgement = 0.1;

        if (over_2pi_ == 1 && start_yaw_ + 2 * M_PI - epsilon_judgement < yaw && diff.seconds() > 1.0)
        {
            rotation_counter_++;
            is_first_counter_ = true;
            last_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Count up");

            std_msgs::msg::Float32 rotation_counter_msg;
            rotation_counter_msg.data = rotation_counter_;
            rotation_counter_pub_->publish(rotation_counter_msg);
        }
        else if (over_2pi_ == -1 && start_yaw_ - 2 * M_PI + epsilon_judgement > yaw && diff.seconds() > 1.0)
        {
            rotation_counter_--;
            is_first_counter_ = true;
            last_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Count down");

            std_msgs::msg::Float32 rotation_counter_msg;
            rotation_counter_msg.data = rotation_counter_;
            rotation_counter_pub_->publish(rotation_counter_msg);
        }
    }

    void angular_velocity_callback(const nokolat2024_msg::msg::Rpy::SharedPtr msg)
    {
        yaw_speed_history_.push_back(msg->yaw);
        if (yaw_speed_history_.size() > 10)
        {
            yaw_speed_history_.pop_front();
        }

        rotation_speed_ave_ = std::accumulate(yaw_speed_history_.begin(), yaw_speed_history_.end(), 0.0) / yaw_speed_history_.size();

        if (rotation_speed_ave_ > 0)
        {
            rotation_direction_ = 1; // 値が増加するのは左旋回
        }
        else
        {
            rotation_direction_ = -1; // 値が減少するのは右旋回
        }
    }

    void counter_reset_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (msg->data == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Reset counter");
            rotation_counter_ = 0;
            is_first_counter_ = true;
        }
    }

    rclcpp::Subscription<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Rpy>::SharedPtr angular_velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr counter_reset_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rotation_counter_pub_;

    double start_yaw_;

    double rotation_speed_ave_;
    double rotation_direction_;

    double rotation_counter_;
    double epsilon_;

    std::deque<double> yaw_speed_history_;

    double yaw_offset_;
    double previous_yaw_;
    int over_2pi_;
    bool is_first_counter_;

    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotationCounterNode>());
    rclcpp::shutdown();
    return 0;
}