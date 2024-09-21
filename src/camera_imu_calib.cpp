#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/qos.hpp>

class IMUCalibNode : public rclcpp::Node
{
public:
    IMUCalibNode() : Node("imu_calib_node")
    {
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/camera/accel/sample", 10, std::bind(&IMUCalibNode::imuCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double roll, pitch, yaw;

        // Calculate roll and pitch from accelerometer data
        roll = atan2(msg->linear_acceleration.y, msg->linear_acceleration.z);
        pitch = atan(-msg->linear_acceleration.x / sqrt(msg->linear_acceleration.y * msg->linear_acceleration.y + msg->linear_acceleration.z * msg->linear_acceleration.z));

        // Yaw cannot be determined from accelerometer data alone
        yaw = 0.0;

        RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUCalibNode>());
    rclcpp::shutdown();
    return 0;
}