#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nokolat2024_msg/msg/rpy.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Tf2RpyNode : public rclcpp::Node
{
public:
    Tf2RpyNode()
        : Node("tf_2_rpy_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // base_linkが利用可能になるまで待機
        while (rclcpp::ok())
        {
            try
            {
                tf_buffer_->lookupTransform("base_link_projected", "base_link", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "base_link is now available.");
                break;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for base_link to become available: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Tf2RpyNode::timer_callback, this));

        rpy_pub_ = this->create_publisher<nokolat2024_msg::msg::Rpy>("roll", 10);
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("base_link", "base_link_projected", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        nokolat2024_msg::msg::Rpy roll_msg;

        roll_msg.roll = roll;
        roll_msg.pitch = pitch;
        roll_msg.yaw = yaw;

        // パブリッシュ
        rpy_pub_->publish(roll_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tf2RpyNode>());
    rclcpp::shutdown();
    return 0;
}