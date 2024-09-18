#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_position_topic_name", "/position");
        this->declare_parameter<std::string>("output_position_stamped_topic_name", "/path");

        // パラメータの取得
        std::string input_position_topic_name;
        this->get_parameter("input_position_topic_name", input_position_topic_name);
        std::string output_position_stamped_topic_name;
        this->get_parameter("output_position_stamped_topic_name", output_position_stamped_topic_name);

        subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            input_position_topic_name, 10, std::bind(&PathPublisher::point_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_position_stamped_topic_name, 10);
    }

private:
    void point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (is_right_pos(*msg))
        {
            geometry_msgs::msg::PointStamped pose;
            pose.header.frame_id = "camera_link";
            pose.header.stamp = this->get_clock()->now();
            pose.point = *msg;
            publisher_->publish(pose);
        }
    }

    bool is_right_pos(const geometry_msgs::msg::Point &point)
    {
        if (point.x > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;
    geometry_msgs::msg::Point pyramid_vertices_[5];
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}