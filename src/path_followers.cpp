#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), current_target_index_(0)
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "path", 10, std::bind(&PathFollower::path_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PathFollower::update_target, this));
    }

private:
    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        path_ = msg->poses;
        current_target_index_ = 0; // Reset target index when a new path is received
    }

    void update_target()
    {
        if (path_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path is empty");
            return;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.pose.position.x = transformStamped.transform.translation.x;
        current_pose.pose.position.y = transformStamped.transform.translation.y;
        current_pose.pose.position.z = transformStamped.transform.translation.z;
        current_pose.pose.orientation = transformStamped.transform.rotation;

        while (current_target_index_ < path_.size())
        {
            auto target_pose = path_[current_target_index_];
            double dx = target_pose.position.x - current_pose.pose.position.x;
            double dy = target_pose.position.y - current_pose.pose.position.y;

            double distance = std::sqrt(dx * dx + dy * dy);
            double angle_to_target = std::atan2(dy, dx);

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            double angle_diff = std::fabs(angle_to_target - yaw);

            // 目標点が機体前方1m以内かつ角度差が30度以内、または目標点が機体の後方にある場合に次の目標点に切り替える
            if ((distance < 1.0 && angle_diff < M_PI / 6) || angle_diff > M_PI / 2)
            {
                current_target_index_++;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Target Position: [%.2f, %.2f, %.2f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Target Orientation: [%.2f, %.2f, %.2f, %.2f]", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
                follow_target(target_pose, current_pose.pose);
                break;
            }
        }

        if (current_target_index_ >= path_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All targets reached");
            stop_robot();
        }
    }

    void follow_target(const geometry_msgs::msg::Pose &target_pose, const geometry_msgs::msg::Pose &current_pose)
    {
        geometry_msgs::msg::Twist cmd_vel;

        double dx = target_pose.position.x - current_pose.position.x;
        double dy = target_pose.position.y - current_pose.position.y;
        double dz = target_pose.position.z - current_pose.position.z;

        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        double angle_to_target = std::atan2(dy, dx);
        double pitch_to_target = std::atan2(dz, std::sqrt(dx * dx + dy * dy));

        tf2::Quaternion q(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double yaw_diff = angle_to_target - yaw;
        double pitch_diff = pitch_to_target - pitch;

        cmd_vel.linear.x = 0.5 * distance;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = current_pose.position.z;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = pitch_diff;
        cmd_vel.angular.z = yaw_diff;

        cmd_vel_publisher_->publish(cmd_vel);
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.angular.y = 0;
        cmd_vel_publisher_->publish(cmd_vel);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<geometry_msgs::msg::Pose> path_;
    size_t current_target_index_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}