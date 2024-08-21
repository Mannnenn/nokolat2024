#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher() : Node("camera_info_publisher")
    {
        // YAMLファイルのパスを取得する
        this->declare_parameter<std::string>("yaml_file", "/param/camera_param.yaml");
        std::string yaml_file_path;
        this->get_parameter("yaml_file", yaml_file_path);

        // YAMLファイルを読み込む
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        auto camera_info = config["camera_info"];

        msg.header.frame_id = camera_info["camera_name"].as<std::string>();
        msg.height = camera_info["height"].as<int>();
        msg.width = camera_info["width"].as<int>();
        msg.distortion_model = camera_info["distortion_model"].as<std::string>();
        msg.d = camera_info["D"].as<std::vector<double>>();
        msg.k = camera_info["K"].as<std::array<double, 9>>();
        msg.r = camera_info["R"].as<std::array<double, 9>>();
        msg.p = camera_info["P"].as<std::array<double, 12>>();

        publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CameraInfoPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "CameraInfoPublisher ノードが開始されました");
    }

private:
    void timer_callback()
    {
        publisher->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "カメラ情報をパブリッシュしました");
    }

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::CameraInfo msg;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}