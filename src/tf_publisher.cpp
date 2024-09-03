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
        this->declare_parameter<std::string>("input_altitude_stamped_topic_name", "/altitude_stamped");
        this->declare_parameter<std::string>("input_quaternion_topic_name", "/quaternion");

        // パラメータの取得
        std::string input_position_topic_name;
        this->get_parameter("input_position_topic_name", input_position_topic_name);
        std::string input_altitude_stamped_topic_name;
        this->get_parameter("input_altitude_stamped_topic_name", input_altitude_stamped_topic_name);
        std::string input_quaternion_topic_name;
        this->get_parameter("input_quaternion_topic_name", input_quaternion_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        point_stamped_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_position_topic_name, qos, std::bind(&ImuToTfNode::point_stamped_callback, this, std::placeholders::_1));
        altitude_stamped_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_altitude_stamped_topic_name, qos, std::bind(&ImuToTfNode::altitude_stamped_callback, this, std::placeholders::_1));
        quaternion_subscriber_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            input_quaternion_topic_name, qos, std::bind(&ImuToTfNode::pose_callback, this, std::placeholders::_1));

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 0,0で初期化してカメラの位置推定がなくても動作するようにする
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
    }

    void pose_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
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
                transform_stamped.child_frame_id = "imu_base_link";
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
            // クォータニオンの差が大きい場合は外れ値として扱う
            if (!is_first_tf_pub_message)
            {
                double diff = quaternion_difference(*msg, previous_orientation);
                if (diff > QUATERNION_DIFF_THRESHOLD)
                {
                    // しきい値よりも、現在のフレームと前のフレームとの差の二乗和が大きい場合は外れ値として無視する
                    // RCLCPP_WARN(this->get_logger(), "Detected outlier quaternion, ignoring this IMU message.");
                    if (ignore > 100)
                    {
                        ignore = 0;
                    }
                    else
                    {
                        ignore++;
                        return;
                    }
                }
            }

            // パブリッシュする値と比較に使う値を更新
            tf2::Quaternion imu_base_link_to_base_link_rotation, transformed_quaternion;
            imu_base_link_to_base_link_rotation.setX(msg->x);
            imu_base_link_to_base_link_rotation.setY(msg->y);
            imu_base_link_to_base_link_rotation.setZ(msg->z);
            imu_base_link_to_base_link_rotation.setW(msg->w);

            // imu_base_link座標系からbase_link座標系への回転を計算
            transformed_quaternion = accumulated_orientation_ * imu_base_link_to_base_link_rotation;

            orientation_.x = transformed_quaternion.x();
            orientation_.y = transformed_quaternion.y();
            orientation_.z = transformed_quaternion.z();
            orientation_.w = transformed_quaternion.w();

            previous_orientation = *msg;
            is_first_tf_pub_message = false;

            // 前のクォータニオンの値を更新
            previous_orientation = *msg;

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = this->now();
            // tf.header.frame_id = "imu_base_link";
            tf.header.frame_id = "map";
            tf.child_frame_id = "base_link";
            tf.transform.translation.x = position_vector_.getX();
            tf.transform.translation.y = position_vector_.getY();
            tf.transform.translation.z = position_vector_.getZ();
            tf.transform.rotation.x = orientation_.x;
            tf.transform.rotation.y = orientation_.y;
            tf.transform.rotation.z = orientation_.z;
            tf.transform.rotation.w = orientation_.w;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    void altitude_stamped_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        position_vector_.setZ(msg->point.z);
    }

    double quaternion_difference(const geometry_msgs::msg::Quaternion &q1, const geometry_msgs::msg::Quaternion &q2)
    {
        return std::sqrt(
            std::pow(q1.x - q2.x, 2) +
            std::pow(q1.y - q2.y, 2) +
            std::pow(q1.z - q2.z, 2) +
            std::pow(q1.w - q2.w, 2));
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_stamped_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitude_stamped_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_subscriber_;

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

    // 最初のメッセージは外れ値として扱わない
    bool is_first_tf_pub_message = true;

    // 外れ値を検出するための閾値
    const double QUATERNION_DIFF_THRESHOLD = 0.1;

    // 外れ値除去された回数をカウントする
    int ignore = 0;

    // 前のクォータニオンの値を保持する変数
    geometry_msgs::msg::Quaternion previous_orientation;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToTfNode>());
    rclcpp::shutdown();
    return 0;
}