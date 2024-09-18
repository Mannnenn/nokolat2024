#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class CubePublisher : public rclcpp::Node
{
public:
    CubePublisher()
        : Node("cube_publisher"), index_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/infra_cam/position", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&CubePublisher::publish_point, this));
        // 立方体の内部をまんべんなく埋める座標を設定
        double step = 1;    // ステップサイズ
        double size = 10.0; // 立方体の一辺の長さ
        for (double x = -size; x <= size; x += step)
        {
            for (double y = -size; y <= size; y += step)
            {
                for (double z = -size; z <= size; z += step)
                {
                    geometry_msgs::msg::Point point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    points_.push_back(point);
                }
            }
        }
    }

private:
    void publish_point()
    {
        if (index_ < points_.size())
        {
            auto message = geometry_msgs::msg::Point();
            message.x = points_[index_].x;
            message.y = points_[index_].y;
            message.z = points_[index_].z;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%f, %f, %f'", message.x, message.y, message.z);
            publisher_->publish(message);
            index_++;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::Point> points_;
    size_t index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubePublisher>());
    rclcpp::shutdown();
    return 0;
}