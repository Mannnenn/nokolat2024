#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"

#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>

class ImuToTfNode : public rclcpp::Node
{
public:
    ImuToTfNode() : Node("imu_to_tf_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), initial_alignment_done_(false)
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_position_topic_name", "/position_stamped");
        this->declare_parameter<std::string>("input_imu_tof_topic_name", "/imu_tof");

        // パラメータの取得
        std::string input_position_topic_name;
        this->get_parameter("input_position_topic_name", input_position_topic_name);
        std::string input_imu_tof_topic_name;
        this->get_parameter("input_imu_tof_topic_name", input_imu_tof_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            input_imu_tof_topic_name, qos, std::bind(&ImuToTfNode::pose_callback, this, std::placeholders::_1));
        point_stamped_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_position_topic_name, qos, std::bind(&ImuToTfNode::point_stamped_callback, this, std::placeholders::_1));

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 0,0で初期化してカメラの位置推定がなくても動作するようにする

        position_vector_.setX(0.0);
        position_vector_.setY(0.0);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
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
                // 1秒間IMUの姿勢データを蓄積する
                tf2::Quaternion orientation;
                orientation.setX(msg->orientation.x);
                orientation.setY(msg->orientation.y);
                orientation.setZ(msg->orientation.z);
                orientation.setW(msg->orientation.w);

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
            // IMUの姿勢をbase_linkに変換して配信(最初の数秒をのぞいて)
            orientation_ = msg->orientation;

            // ToFセンサのz座標を取得し、姿勢による分を補正する
            double roll, pitch, yaw;
            double raw_z = msg->position.z;
            tf2::Quaternion q(orientation_.x, orientation_.y, orientation_.z, orientation_.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            double vertical_distance = calculate_vertical_distance(raw_z, roll, pitch);
            position_vector_.setZ(low_pass_filter(vertical_distance));

            // printf("z_translation_: %f\n", z_translation_);

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = this->now();
            tf.header.frame_id = "imu_link";
            tf.child_frame_id = "base_link";
            tf.transform.translation.x = position_vector_.getX();
            tf.transform.translation.y = position_vector_.getY();
            tf.transform.translation.z = position_vector_.getZ();
            tf.transform.rotation.x = msg->orientation.x;
            tf.transform.rotation.y = msg->orientation.y;
            tf.transform.rotation.z = msg->orientation.z;
            tf.transform.rotation.w = msg->orientation.w;
            tf_broadcaster_->sendTransform(tf);
        }
    }

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
    }

    double calculate_vertical_distance(double z_translation_, double roll, double pitch)
    {
        // Assuming z_translation_ is the hypotenuse of a right triangle,
        // and roll is the angle between the hypotenuse and the vertical side
        return z_translation_ * cos(roll) * cos(pitch);
    }

    float low_pass_filter(float z_current)
    {
        float alpha = 0.5; // 適切な値に調整する
        float z_filtered = z_filtered_prev_ * (1 - alpha) + z_current * alpha;
        z_filtered_prev_ = z_filtered; // 更新された値を保存
        return z_filtered;
    }

    double z_filtered_prev_ = 0.0;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_stamped_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::Quaternion orientation_;
    tf2::Vector3 position_vector_;

    rclcpp::Time start_time_;
    bool initial_alignment_done_;
    tf2::Quaternion accumulated_orientation_;
    int sample_count_ = 0;
    bool first_callback = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToTfNode>());
    rclcpp::shutdown();
    return 0;
}