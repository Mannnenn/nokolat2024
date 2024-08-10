#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        path_.header.frame_id = "base_link";
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PathPublisher::publishPath, this));
    }

private:
    void publishPath() {
        geometry_msgs::msg::TransformStamped transform;
        try {
            // TFデータを取得
            transform = tf_buffer_.lookupTransform("base_link", "imu_link", tf2::TimePointZero);

            // TFデータを基にポーズを更新
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "base_link";
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;

            // 前回のポーズからの距離を計算
            if (!path_.poses.empty()) {
                const auto& last_pose = path_.poses.back().pose.position;
                double distance = std::sqrt(
                    std::pow(pose.pose.position.x - last_pose.x, 2) +
                    std::pow(pose.pose.position.y - last_pose.y, 2) +
                    std::pow(pose.pose.position.z - last_pose.z, 2)
                );

                // 一定の距離以上移動した場合にのみポーズを追加
                if (distance < 0.5) { // 0.1メートル以上移動した場合
                    return;
                }
            }

            // パスにポーズを追加
            path_.poses.push_back(pose);

            // パスをパブリッシュ
            path_pub_->publish(path_);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    nav_msgs::msg::Path path_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}