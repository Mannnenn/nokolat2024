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

        // で初期化してカメラの位置推定がなくても動作するようにする
        position_vector_.setX(37);
        position_vector_.setY(-1);
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

                map2imu_base_link_rotation_ = accumulated_orientation_;
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
            // クォータニオンが外れ値でないかを確認
            // 　仮定：z軸が上方向を向いている
            if (!isZAxisUp(*msg))
            {
                // RCLCPP_INFO(this->get_logger(), "Quaternion is invalid. Ignoring... %f", z_axis_.z());
                return;
            }
            // パブリッシュする値と比較に使う値を更新
            tf2::Quaternion base_link_rotation, transformed_quaternion;
            base_link_rotation.setX(msg->x);
            base_link_rotation.setY(msg->y);
            base_link_rotation.setZ(msg->z);
            base_link_rotation.setW(msg->w);

            // imu_base_link座標系からbase_link座標系への回転を計算
            transformed_quaternion = base_link_rotation.inverse() * map2imu_base_link_rotation_;

            orientation_.x = transformed_quaternion.x();
            orientation_.y = transformed_quaternion.y();
            orientation_.z = transformed_quaternion.z();
            // 回転方向を逆にする
            orientation_.w = -transformed_quaternion.w();

            send_tf();
        }
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

    // クォータニオンからz軸方向を取得する関数
    bool isZAxisUp(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        z_axis_ = m.getColumn(2); // z軸方向を取得

        // z軸が上方向を向いているかを確認
        return z_axis_.z() > 0;
    }

    void send_tf()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = position_vector_.getX();
        transform_stamped.transform.translation.y = position_vector_.getY();
        transform_stamped.transform.translation.z = position_vector_.getZ();
        transform_stamped.transform.rotation = orientation_;

        tf_broadcaster_->sendTransform(transform_stamped);
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
    tf2::Quaternion map2imu_base_link_rotation_;
    int sample_count_ = 0;
    bool first_callback = true;

    tf2::Vector3 z_axis_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToTfNode>());
    rclcpp::shutdown();
    return 0;
}