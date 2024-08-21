#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

class depthEstimateNode : public rclcpp::Node
{
public:
    depthEstimateNode() : Node("stereo_matching_node")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_left_cog_topic_name", "/left/cog_raw");
        this->declare_parameter<std::string>("input_right_cog_topic_name", "/right/cog_raw");
        this->declare_parameter<std::string>("output_depth_topic_name", "/depth");

        // パラメータの取得
        std::string input_left_cog_topic_name;
        this->get_parameter("input_left_cog_topic_name", input_left_cog_topic_name);
        std::string input_right_cog_topic_name;
        this->get_parameter("input_right_cog_topic_name", input_right_cog_topic_name);
        std::string output_depth_topic_name;
        this->get_parameter("output_depth_topic_name", output_depth_topic_name);

        left_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            input_left_cog_topic_name, 10, std::bind(&depthEstimateNode::leftCogCallback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            input_right_cog_topic_name, 10, std::bind(&depthEstimateNode::rightCogCallback, this, std::placeholders::_1));

        depth_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_depth_topic_name, 10);

        // YAMLファイルのパスを取得する
        this->declare_parameter<std::string>("yaml_right_file", "/param/camera_param_right.yaml");
        std::string yaml_right_file_path;
        this->get_parameter("yaml_right_file", yaml_right_file_path);

        try
        {
            // YAMLファイルを読み込む
            YAML::Node config_right = YAML::LoadFile(yaml_right_file_path);
            if (!config_right["camera_info_right"])
            {
                throw std::runtime_error("camera_info_right not found in " + yaml_right_file_path);
            }
            camera_info_right = config_right["camera_info_right"];
        }
        catch (const YAML::BadFile &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
            return;
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error: %s", e.what());
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            return;
        }

        focal_length = camera_info_right["K"][0].as<double>();

        double p3 = camera_info_right["P"][3].as<double>();
        double p7 = camera_info_right["P"][7].as<double>();
        double p11 = camera_info_right["P"][11].as<double>();

        // ノルムの計算
        double norm = std::sqrt(std::pow(p3, 2) + std::pow(p7, 2) + std::pow(p11, 2));

        // ミリメートルからメートルに変換
        baseline = norm / 1000.0; // mm to m
    }

private:
    void leftCogCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        left_cog = msg->x;
        processDepth();
    }

    void rightCogCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        right_cog = msg->x;
    }

    void processDepth()
    {
        if (std::isnan(left_cog) || std::isnan(right_cog))
        {
            // RCLCPP_WARN(this->get_logger(), "left_cog or right_cog is NaN");
            return;
        }

        // 左右カメラの差を計算
        float disparity = left_cog - right_cog;

        // ゼロ除算を回避
        if (disparity == 0.0f)
        {
            RCLCPP_WARN(this->get_logger(), "Disparity is zero, cannot compute depth");
            return;
        }

        // 視差から奥行きを計算
        float depth = focal_length * baseline / disparity;

        std_msgs::msg::Float32 depth_msg;

        depth_msg.data = static_cast<float>(depth);
        depth_pub_->publish(depth_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr left_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr right_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;

    YAML::Node camera_info_right;

    float left_cog;
    float right_cog;

    float focal_length;
    float baseline;

    float low_pass_filter(float depth_current)
    {
        float alpha = 0.1; // 適切な値に調整する
        float depth_filtered = depth_filtered_prev_ * (1 - alpha) + depth_current * alpha;
        depth_filtered_prev_ = depth_filtered; // 更新された値を保存
        return depth_filtered;
    }

    float depth_filtered_prev_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<depthEstimateNode>());
    rclcpp::shutdown();
    return 0;
}