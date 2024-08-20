#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <yaml-cpp/yaml.h>

class positionEstimateNode : public rclcpp::Node {
public:
        positionEstimateNode() : Node("stereo_matching_node") {
        // パラメータの宣言
        // パラメータの宣言
        this->declare_parameter<std::string>("input_left_cog_topic_name", "/left/cog_raw");
        this->declare_parameter<std::string>("input_right_cog_topic_name", "/right/cog_raw");
        this->declare_parameter<std::string>("input_depth_topic_name", "/depth");
        this->declare_parameter<std::string>("output_position_topic_name", "/position");

        // パラメータの取得
        std::string input_left_cog_topic_name;
        this->get_parameter("input_left_cog_topic_name", input_left_cog_topic_name);
        std::string input_right_cog_topic_name;
        this->get_parameter("input_right_cog_topic_name", input_right_cog_topic_name);
        std::string input_depth_topic_name;
        this->get_parameter("input_depth_topic_name", input_depth_topic_name);
        std::string output_position_topic_name;
        this->get_parameter("output_position_topic_name", output_position_topic_name);


        left_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        input_left_cog_topic_name, 10, std::bind(&positionEstimateNode::leftCogCallback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        input_right_cog_topic_name, 10, std::bind(&positionEstimateNode::rightCogCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(input_depth_topic_name, 10, std::bind(&positionEstimateNode::depthCallback, this, std::placeholders::_1));

        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(output_position_topic_name, 10);

        // YAMLファイルのパスを取得する
        this->declare_parameter<std::string>("yaml_right_file", "/param/camera_param_right.yaml");
        std::string yaml_right_file_path;
        this->get_parameter("yaml_right_file", yaml_right_file_path);


        try {
            // YAMLファイルを読み込む
            YAML::Node config_right = YAML::LoadFile(yaml_right_file_path);
            if (!config_right["camera_info_right"]) {
                throw std::runtime_error("camera_info_right not found in " + yaml_right_file_path);
            }
            camera_info_right = config_right["camera_info_right"];
        } catch (const YAML::BadFile& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
            return;
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Runtime error: %s", e.what());
            return;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            return;
        }

        focal_length = camera_info_right["K"][0].as<double>();

        center_x = camera_info_right["K"][2].as<double>();
        center_y = camera_info_right["K"][5].as<double>();
    }

private:
    void leftCogCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        position_left_ = *msg;
    }

    void rightCogCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        position_right_ = *msg;
    }

    void depthCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        depth_ = msg->data;
        processPosition();
    }

    void processPosition() {
        if (std::isnan(depth_)) {
            RCLCPP_WARN(this->get_logger(), "left_cog or right_cog is NaN");
            return;
        }

        float x = (position_left_.x + position_right_.x) / 2;
        float y = (position_left_.y + position_right_.y) / 2;


        // calc position,In Realsense D455, the depth is the distance from the camera to the object.




        position_.y = -(x - center_x) * depth_ / focal_length;//left as y axis.
        position_.z = (y - center_y) * depth_ / focal_length; //up as z axis.
        position_.x = depth_; //forward as x axis.

        //printf("x: %f, y: %f\n", position_.x, position_.z);

        position_pub_->publish(position_);
        }

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr left_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr right_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;

        geometry_msgs::msg::Point position_left_;
        geometry_msgs::msg::Point position_right_;

        float focal_length;
        float center_x;
        float center_y;

        float depth_;

        YAML::Node camera_info_right;

        geometry_msgs::msg::Point position_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<positionEstimateNode>());
    rclcpp::shutdown();
    return 0;
}