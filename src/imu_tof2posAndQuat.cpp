#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <deque>

class ImuToF2PoseQuat : public rclcpp::Node
{
public:
    ImuToF2PoseQuat() : Node("path_publisher")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_imu_tof_topic_name", "/imu_tof");
        this->declare_parameter<std::string>("output_altitude_stamped_topic_name", "/altitude_stamped");
        this->declare_parameter<std::string>("output_quaternion_topic_name", "/quaternion");

        // パラメータの取得
        std::string input_imu_tof_topic_name;
        this->get_parameter("input_imu_tof_topic_name", input_imu_tof_topic_name);
        std::string output_altitude_stamped_topic_name;
        this->get_parameter("output_altitude_stamped_topic_name", output_altitude_stamped_topic_name);
        std::string output_quaternion_topic_name;
        this->get_parameter("output_quaternion_topic_name", output_quaternion_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            input_imu_tof_topic_name, qos, std::bind(&ImuToF2PoseQuat::pose_callback, this, std::placeholders::_1));

        publisher_point_stamped_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_altitude_stamped_topic_name, 10);
        publisher_quaternion_ = this->create_publisher<geometry_msgs::msg::Quaternion>(output_quaternion_topic_name, 10);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::Quaternion quat;
        quat.x = msg->orientation.x;
        quat.y = msg->orientation.y;
        quat.z = msg->orientation.z;
        quat.w = msg->orientation.w;

        publisher_quaternion_->publish(quat);

        // ToFセンサの値を取得し、姿勢による分を補正する
        double roll, pitch, yaw;
        double raw_z = msg->position.z;
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        geometry_msgs::msg::PointStamped pose;
        pose.header.frame_id = "base_link";
        pose.header.stamp = this->now();
        z_history_.push_back(calculate_vertical_distance(raw_z, roll, pitch));
        if (z_history_.size() > 100)
        {
            z_history_.pop_front();
        }
        pose.point.z = std::accumulate(z_history_.begin(), z_history_.end(), 0.0) / z_history_.size();

        publisher_point_stamped_->publish(pose);
    }

    double calculate_vertical_distance(double z_raw, double roll, double pitch)
    {
        // Assuming z_translation_ is the hypotenuse of a right triangle,
        // and roll is the angle between the hypotenuse and the vertical side
        return z_raw * cos(roll) * cos(pitch);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_point_stamped_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_quaternion_;

    std::deque<double> z_history_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToF2PoseQuat>());
    rclcpp::shutdown();
    return 0;
}