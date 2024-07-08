#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cmath>



class TofSmoothNode : public rclcpp::Node
{
public:
    TofSmoothNode() : Node("imu_to_tf_node")
    {
        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_tof", qos, std::bind(&TofSmoothNode::imu_callback, this, std::placeholders::_1));
        altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>("altitude", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std_msgs::msg::Float32 altitude_msg;
        float z = msg->angular_velocity_covariance[0]; // 仮の値
        altitude_msg.data = low_pass_filter(z);
        altitude_publisher_->publish(altitude_msg);
        // ここでフィルタ処理されたzを使用
    }

    float low_pass_filter(float z_current)
    {
        float alpha = 0.9; // 適切な値に調整する
        float z_filtered = z_filtered_prev_ * (1 - alpha) + z_current * alpha;
        z_filtered_prev_ = z_filtered; // 更新された値を保存
        return z_filtered;
    }

    float z_filtered_prev_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TofSmoothNode>());
    rclcpp::shutdown();
    return 0;
}

