#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cmath>



class ImuToTfNode : public rclcpp::Node
{
public:
    ImuToTfNode() : Node("imu_to_tf_node")
    {
        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_tof", qos, std::bind(&ImuToTfNode::imu_callback, this, std::placeholders::_1));
        altitude_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "altitude", qos, std::bind(&ImuToTfNode::altitude_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        geometry_msgs::msg::TransformStamped tf;

        double roll, pitch;
        quaternion_to_roll_pitch(msg->orientation, roll, pitch);
        double vertical_distance = calculate_vertical_distance(z_translation_, roll, pitch);
        //RCLCPP_INFO(this->get_logger(), "Vertical distance: %f", vertical_distance);


        tf.header.stamp = this->now();
        tf.header.frame_id = "base_link";
        tf.child_frame_id = "imu_link";
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = low_pass_filter(vertical_distance);
        tf.transform.rotation = msg->orientation;

        tf_broadcaster_->sendTransform(tf);
    }

    void altitude_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        z_translation_ = msg->data;
    }

    void quaternion_to_roll_pitch(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch)
    {
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);
    }

    double calculate_vertical_distance(double z_translation_, double roll,double pitch)
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

    float z_filtered_prev_;

    double z_translation_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToTfNode>());
    rclcpp::shutdown();
    return 0;
}

