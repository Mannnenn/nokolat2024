#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

class DropPublisher : public rclcpp::Node
{
public:
    DropPublisher()
        : Node("drop_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("output_drop_timing_topic_name", "/drop");
        this->declare_parameter<std::string>("output_throttle_off_timing_topic_name", "/throttle_off");

        // パラメータの取得
        std::string output_drop_timing_topic_name;
        this->get_parameter("output_drop_timing_topic_name", output_drop_timing_topic_name);
        std::string output_throttle_off_timing_topic_name;
        this->get_parameter("output_throttle_off_timing_topic_name", output_throttle_off_timing_topic_name);

        publisher_drop_ = this->create_publisher<std_msgs::msg::String>(output_drop_timing_topic_name, 10);
        publisher_throttle_off_ = this->create_publisher<std_msgs::msg::String>(output_throttle_off_timing_topic_name, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DropPublisher::on_timer, this));
    }

private:
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("drop_point", "base_link", tf2::TimePointZero);
            if (0.0 < transformStamped.transform.translation.x && transformStamped.transform.translation.x < 2)
            {
                auto message = std_msgs::msg::String();
                message.data = "drop";
                publisher_drop_->publish(message);
            }
            else
            {
                auto message = std_msgs::msg::String();
                message.data = "wait";
                publisher_drop_->publish(message);
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to drop_point: %s", ex.what());
        }

        try
        {
            transformStamped = tf_buffer_.lookupTransform("throttle_off_point", "base_link", tf2::TimePointZero);
            if (0 < transformStamped.transform.translation.x)
            {
                auto message = std_msgs::msg::String();
                message.data = "throttle_off";
                publisher_throttle_off_->publish(message);
            }
            else
            {
                auto message = std_msgs::msg::String();
                message.data = "wait";
                publisher_throttle_off_->publish(message);
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to throttle_off_point: %s", ex.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_drop_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_throttle_off_;

    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DropPublisher>());
    rclcpp::shutdown();
    return 0;
}