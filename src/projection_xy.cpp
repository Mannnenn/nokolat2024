#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TfProjectionNode : public rclcpp::Node
{
public:
    TfProjectionNode()
        : Node("tf_projection_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // base_linkが利用可能になるまで待機
        while (rclcpp::ok())
        {
            try
            {
                tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
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
            std::chrono::milliseconds(100),
            std::bind(&TfProjectionNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        // 回転を取得
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // xy平面に投影
        tf2::Quaternion q_xy;

        // 新しい変換を作成
        geometry_msgs::msg::TransformStamped transform_stamped_xy;
        transform_stamped_xy.header.stamp = this->now();
        transform_stamped_xy.header.frame_id = "base_link";
        transform_stamped_xy.child_frame_id = "base_link_xy";
        transform_stamped_xy.transform.translation = transform_stamped.transform.translation;
        transform_stamped_xy.transform.rotation.x = q_xy.x();
        transform_stamped_xy.transform.rotation.y = q_xy.y();
        transform_stamped_xy.transform.rotation.z = q_xy.z();
        transform_stamped_xy.transform.rotation.w = q_xy.w();

        // パブリッシュ
        tf_broadcaster_->sendTransform(transform_stamped_xy);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfProjectionNode>());
    rclcpp::shutdown();
    return 0;
}