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

        // パラメータの取得
        std::string output_drop_timing_topic_name;
        this->get_parameter("output_drop_timing_topic_name", output_drop_timing_topic_name);

        publisher_ = this->create_publisher<std_msgs::msg::String>(output_drop_timing_topic_name, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25),
            std::bind(&DropPublisher::on_timer, this));
    }

private:
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("waypoint_2", "base_link", tf2::TimePointZero);
            if (0.0 < transformStamped.transform.translation.x && transformStamped.transform.translation.x < 2)
            {
                auto message = std_msgs::msg::String();
                message.data = "drop";
                publisher_->publish(message);
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to waypoint_2: %s", ex.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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