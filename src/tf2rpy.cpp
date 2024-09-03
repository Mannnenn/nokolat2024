#include <rclcpp/rclcpp.hpp>

#include <deque>

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

        rpy_pub_ = this->create_publisher<nokolat2024_msg::msg::Rpy>("rpy", 10);
        angular_velocity_pub_ = this->create_publisher<nokolat2024_msg::msg::Rpy>("angular_velocity", 10);
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

        nokolat2024_msg::msg::Rpy rpy_msg;

        rpy_msg.roll = roll;
        rpy_msg.pitch = pitch;
        rpy_msg.yaw = yaw;

        // パブリッシュ
        rpy_pub_->publish(rpy_msg);

        // 履歴を保存
        roll_history_.push_back(roll);
        pitch_history_.push_back(pitch);
        yaw_history_.push_back(yaw);

        rclcpp::Time current_time = this->now();
        if (last_time_.seconds() == 0)
        {
            last_time_ = current_time;
            return;
        }

        rclcpp::Duration diff = current_time - last_time_;

        // 　中心差分法で角速度を計算
        if (roll_history_.size() > 3)
        {
            double roll_diff = (roll_history_[2] - roll_history_[0]) / (2 * diff.seconds());
            double pitch_diff = (pitch_history_[2] - pitch_history_[0]) / (2 * diff.seconds());
            double yaw_diff = (yaw_history_[2] - yaw_history_[0]) / (2 * diff.seconds());

            RCLCPP_INFO(this->get_logger(), "roll_diff: %f, pitch_diff: %f, yaw_diff: %f", roll_diff, pitch_diff, yaw_diff);

            // 履歴を削除
            roll_history_.pop_front();
            pitch_history_.pop_front();
            yaw_history_.pop_front();

            nokolat2024_msg::msg::Rpy angular_velocity_msg;
            angular_velocity_msg.roll = roll_diff;
            angular_velocity_msg.pitch = pitch_diff;
            angular_velocity_msg.yaw = yaw_diff;

            angular_velocity_pub_->publish(angular_velocity_msg);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_pub_;
    rclcpp::Publisher<nokolat2024_msg::msg::Rpy>::SharedPtr angular_velocity_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::deque<double> roll_history_;
    std::deque<double> pitch_history_;
    std::deque<double> yaw_history_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tf2RpyNode>());
    rclcpp::shutdown();
    return 0;
}