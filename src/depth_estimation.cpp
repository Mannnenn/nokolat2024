#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>

class depthEstimateNode : public rclcpp::Node {
public:
        depthEstimateNode() : Node("stereo_matching_node") {
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

    }

private:
    void leftCogCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        left_cog = msg->x;
        processDisparity();
    }

    void rightCogCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        right_cog = msg->x;
        processDisparity();
    }

    void processDisparity() {
        if (std::isnan(left_cog) || std::isnan(right_cog)) {
            //RCLCPP_WARN(this->get_logger(), "left_cog or right_cog is NaN");
            return;
        }

        // 左右カメラの差を計算
        float disparity = left_cog - right_cog;

        // ゼロ除算を回避
        if (disparity == 0.0f) {
            RCLCPP_WARN(this->get_logger(), "Disparity is zero, cannot compute depth");
            return;
        }

        // 視差から奥行きを計算
        float focal_length = 640.0f;
        float baseline = 1.03f;
        float depth = focal_length * baseline / disparity;

        std_msgs::msg::Float32 depth_msg;

        depth_msg.data = static_cast<float>(depth);
        depth_pub_->publish(depth_msg);

        }

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr left_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr right_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
        float left_cog;
        float right_cog;

    float low_pass_filter(float depth_current)
    {
        float alpha = 0.1; // 適切な値に調整する
        float depth_filtered = depth_filtered_prev_ * (1 - alpha) + depth_current * alpha;
        depth_filtered_prev_ = depth_filtered; // 更新された値を保存
        return depth_filtered;
    }

    float depth_filtered_prev_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<depthEstimateNode>());
    rclcpp::shutdown();
    return 0;
}