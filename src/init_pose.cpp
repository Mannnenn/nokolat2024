#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;

class IMUAlignmentNode : public rclcpp::Node
{
public:
    IMUAlignmentNode()
        : Node("imu_alignment_node"), initial_alignment_done_(false)
    {
        // QoS設定を作成
        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/imu", qos, std::bind(&IMUAlignmentNode::imuCallback, this, std::placeholders::_1));

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void imuCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        if (!initial_alignment_done_)
        {

            if (first_callback)
            {
                // スタート時刻を記録
                start_time_ = this->now();

                RCLCPP_INFO(this->get_logger(), "IMU data received.");
                first_callback = false;
            }
            rclcpp::Time current_time = this->now();
            rclcpp::Duration duration = current_time - start_time_;

            if (duration.seconds() < 1.0)
            {
                // 10秒間IMUの姿勢データを蓄積する
                tf2::Quaternion orientation;
                orientation.setX(msg->x);
                orientation.setY(msg->y);
                orientation.setZ(msg->z);
                orientation.setW(msg->w);

                accumulated_orientation_ += orientation;
                sample_count_++;

                // RCLCPP_INFO(this->get_logger(), "Accumulating IMU data... (%d)", sample_count_);
            }
            else
            {
                // 10秒後、IMUの平均姿勢を計算する
                accumulated_orientation_ /= sample_count_;

                // クォータニオンの正規化を行う
                accumulated_orientation_.normalize();

                // Print the accumulated quaternion
                // RCLCPP_INFO(this->get_logger(), "Accumulated IMU orientation: (%f, %f, %f, %f)",
                //            accumulated_orientation_.x(), accumulated_orientation_.y(),
                //            accumulated_orientation_.z(), accumulated_orientation_.w());

                tf2::Quaternion map_to_imu_rotation = accumulated_orientation_.inverse(); // map座標系に対するIMU座標系の回転

                // 変換をブロードキャスト
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = this->now();
                transform_stamped.header.frame_id = "map";
                transform_stamped.child_frame_id = "imu_link";
                transform_stamped.transform.rotation.x = map_to_imu_rotation.x();
                transform_stamped.transform.rotation.y = map_to_imu_rotation.y();
                transform_stamped.transform.rotation.z = map_to_imu_rotation.z();
                transform_stamped.transform.rotation.w = map_to_imu_rotation.w();

                transform_stamped.transform.translation.x = 0.0;
                transform_stamped.transform.translation.y = 0.0;
                transform_stamped.transform.translation.z = 0.0;

                // 静的変換をパブリッシュ
                static_tf_broadcaster_->sendTransform(transform_stamped);
                initial_alignment_done_ = true;

                // RCLCPP_INFO(this->get_logger(), "IMU alignment completed.");
            }
        }
        else
        {
            geometry_msgs::msg::TransformStamped transform_stamped_imu;
            transform_stamped_imu.header.stamp = this->now();
            transform_stamped_imu.header.frame_id = "imu_link";
            transform_stamped_imu.child_frame_id = "base_link";
            transform_stamped_imu.transform.rotation.x = msg->x;
            transform_stamped_imu.transform.rotation.y = msg->y;
            transform_stamped_imu.transform.rotation.z = msg->z;
            transform_stamped_imu.transform.rotation.w = msg->w;

            transform_stamped_imu.transform.translation.x = 0.0;
            transform_stamped_imu.transform.translation.y = 0.0;
            transform_stamped_imu.transform.translation.z = 0.0;

            tf_broadcaster_->sendTransform(transform_stamped_imu);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr imu_subscriber_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Time start_time_;
    bool initial_alignment_done_;
    tf2::Quaternion accumulated_orientation_;
    int sample_count_ = 0;
    bool first_callback = true;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUAlignmentNode>());
    rclcpp::shutdown();
    return 0;
}