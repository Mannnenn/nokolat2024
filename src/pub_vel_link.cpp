#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"

#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

class ImuToTfNode : public rclcpp::Node
{
public:
    ImuToTfNode() : Node("imu_to_tf_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_position_topic_name", "/position_stamped");
        this->declare_parameter<std::string>("input_altitude_stamped_topic_name", "/altitude_stamped");

        // パラメータの取得
        std::string input_position_topic_name;
        this->get_parameter("input_position_topic_name", input_position_topic_name);
        std::string input_altitude_stamped_topic_name;
        this->get_parameter("input_altitude_stamped_topic_name", input_altitude_stamped_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        point_stamped_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_position_topic_name, qos, std::bind(&ImuToTfNode::point_stamped_callback, this, std::placeholders::_1));
        altitude_stamped_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_altitude_stamped_topic_name, qos, std::bind(&ImuToTfNode::altitude_stamped_callback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tf_buffer_w_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_w_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_w_);

        // base_linkが利用可能になるまで待機
        while (rclcpp::ok())
        {
            try
            {
                tf_buffer_w_->lookupTransform("base_link_projected", "base_link", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "base_link is now available.");
                break;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for base_link to become available: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

private:
    void point_stamped_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {

        geometry_msgs::msg::PointStamped transformed_point;
        try
        {
            tf_buffer_.transform(*msg, transformed_point, "map");
            position_vector_.setX(transformed_point.point.x);
            position_vector_.setY(transformed_point.point.y);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        }

        send_tf();
    }

    void altitude_stamped_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // 高度が負の値の場合は無視
        if (msg->point.z < 0.0)
        {
            return;
        }
        position_vector_.setZ(msg->point.z);
    }

    void send_tf()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "vel_link";
        transform_stamped.transform.translation.x = position_vector_.getX();
        transform_stamped.transform.translation.y = position_vector_.getY();
        transform_stamped.transform.translation.z = position_vector_.getZ();
        transform_stamped.transform.rotation.x = 0;
        transform_stamped.transform.rotation.y = 0;
        transform_stamped.transform.rotation.z = 0;
        transform_stamped.transform.rotation.w = 1;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_stamped_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitude_stamped_subscriber_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_w_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_w_;

    tf2::Vector3 position_vector_;

    tf2::Vector3 z_axis_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToTfNode>());
    rclcpp::shutdown();
    return 0;
}