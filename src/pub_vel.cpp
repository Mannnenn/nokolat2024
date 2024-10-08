#include <rclcpp/rclcpp.hpp>

#include <deque>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <std_msgs/msg/float32.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class VelocityCalculator : public rclcpp::Node
{
public:
    VelocityCalculator()
        : Node("velocity_calculator")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<std::string>("input_position_stamped_topic_name", "/point_stamped");
        this->declare_parameter<std::string>("output_twist_topic_name", "/twist");
        this->declare_parameter<std::string>("output_speed_topic_name", "/speed");

        // パラメータの取得
        std::string input_position_stamped_topic_name;
        this->get_parameter("input_position_stamped_topic_name", input_position_stamped_topic_name);
        std::string output_twist_topic_name;
        this->get_parameter("output_twist_topic_name", output_twist_topic_name);
        std::string output_speed_topic_name;
        this->get_parameter("output_speed_topic_name", output_speed_topic_name);

        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_position_stamped_topic_name, 10, std::bind(&VelocityCalculator::pointCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_twist_topic_name, 10);
        publisher_speed_ = this->create_publisher<std_msgs::msg::Float32>(output_speed_topic_name, 10);

        last_time = this->now();
    }

private:
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PointStamped transformed_point;
        try
        {
            tf_buffer_->transform(*msg, transformed_point, "map", tf2::durationFromSec(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        }

        if (!last_point_)
        {
            last_point_ = std::make_shared<geometry_msgs::msg::PointStamped>(transformed_point);
            return;
        }

        // Calculate time difference
        rclcpp::Duration duration = this->now() - last_time;

        double dt = duration.seconds();

        if (dt == 0)
        {
            return;
        }

        point_buffer_.push_back(std::make_shared<geometry_msgs::msg::PointStamped>(transformed_point));

        geometry_msgs::msg::Point delta;

        if (point_buffer_.size() > 5)
        {
            delta.x = (point_buffer_[4]->point.x - point_buffer_[0]->point.x) / (4 * dt) + (point_buffer_[3]->point.x - point_buffer_[1]->point.x) / (2 * dt);
            delta.y = (point_buffer_[4]->point.y - point_buffer_[0]->point.y) / (4 * dt) + (point_buffer_[3]->point.y - point_buffer_[1]->point.y) / (2 * dt);
            delta.z = (point_buffer_[4]->point.z - point_buffer_[0]->point.z) / (4 * dt) + (point_buffer_[3]->point.z - point_buffer_[1]->point.z) / (2 * dt);

            point_buffer_.pop_front();

            // Calculate the norm of the velocity
            double velocity_norm = std::sqrt(
                std::pow(delta.y, 2) +
                std::pow(delta.x, 2) +
                std::pow(delta.z, 2));

            // Check if the velocity norm is too large
            double max_velocity_norm = 15.0; // Define a threshold for maximum velocity norm
            if (velocity_norm > max_velocity_norm)
            {
                // RCLCPP_WARN(this->get_logger(), "Velocity norm too large: %f", velocity_norm);
                last_point_ = std::make_shared<geometry_msgs::msg::PointStamped>(transformed_point);
                last_time = this->now();
                return;
            }

            geometry_msgs::msg::TwistStamped velocity_msg;
            velocity_msg.header.stamp = this->now();
            velocity_msg.header.frame_id = "vel_link";
            velocity_msg.twist.linear.x = delta.x;
            velocity_msg.twist.linear.y = delta.y;
            velocity_msg.twist.linear.z = delta.z;

            publisher_->publish(velocity_msg);

            velocity_buffer_.push_back(velocity_norm);

            if (velocity_buffer_.size() > 10)
            {
                velocity_buffer_.pop_front();
            }

            std_msgs::msg::Float32 speed_msg;
            speed_msg.data = std::accumulate(velocity_buffer_.begin(), velocity_buffer_.end(), 0.0) / velocity_buffer_.size();
            publisher_speed_->publish(speed_msg);

            last_point_ = std::make_shared<geometry_msgs::msg::PointStamped>(transformed_point);
            last_time = this->now();
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_speed_;

    geometry_msgs::msg::PointStamped::SharedPtr last_point_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::deque<geometry_msgs::msg::PointStamped::SharedPtr> point_buffer_;
    std::deque<double> velocity_buffer_;

    rclcpp::Time last_time;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityCalculator>());
    rclcpp::shutdown();
    return 0;
}