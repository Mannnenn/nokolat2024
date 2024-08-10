#include <opencv2/opencv.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

class StereoMatchingNode : public rclcpp::Node {
public:
        StereoMatchingNode() : Node("stereo_matching_node") {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_left_image_topic_name", "/left/image_raw");
        this->declare_parameter<std::string>("input_right_image_topic_name", "/right/image_raw");
        this->declare_parameter<std::string>("output_image_topic_name", "/camera/camera/infra/depth_image");
        this->declare_parameter<std::string>("output_mean_depth_topic_name", "/mean_depth");

        // パラメータの取得
        std::string input_left_image_topic_name;
        this->get_parameter("input_left_image_topic_name", input_left_image_topic_name);
        std::string input_right_image_topic_name;
        this->get_parameter("input_right_image_topic_name", input_right_image_topic_name);
        std::string output_image_topic_name;
        this->get_parameter("output_image_topic_name", output_image_topic_name);
        std::string output_mean_depth_topic_name;
        this->get_parameter("output_mean_depth_topic_name", output_mean_depth_topic_name);


        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_left_image_topic_name, 10, std::bind(&StereoMatchingNode::leftImageCallback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_right_image_topic_name, 10, std::bind(&StereoMatchingNode::rightImageCallback, this, std::placeholders::_1));

        disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_name, 10);
        mean_depth_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_mean_depth_topic_name, 10);

        stereo_bm_ = cv::StereoBM::create();
    }

private:
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        left_image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
        processImages();
    }

    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        right_image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
        processImages();
    }

    void processImages() {
        if (left_image_.empty() || right_image_.empty()) {
            return;
        }

        stereo_bm_->setBlockSize(15);
        stereo_bm_->setNumDisparities(64);
        stereo_bm_->setPreFilterType(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE);
        stereo_bm_->setTextureThreshold(1);

        // ノイズを減らすための追加設定
        stereo_bm_->setPreFilterSize(9); // プリフィルタのサイズを設定
        stereo_bm_->setPreFilterCap(31); // プリフィルタのキャップを設定
        stereo_bm_->setSpeckleRange(16); // スペックル範囲を設定
        stereo_bm_->setSpeckleWindowSize(100); // スペックルウィンドウサイズを設定

        cv::Mat disparity;
        stereo_bm_->compute(left_image_, right_image_, disparity);

        cv::Mat depth_map;
        float focal_length = 640.0f;
        float baseline = 0.95f;
        disparity.convertTo(disparity, CV_32F);
        depth_map = focal_length * baseline / disparity;

        // 無限大の値を無視するためのマスクを作成
        cv::Mat mask = (depth_map != std::numeric_limits<float>::infinity()) & 
               (depth_map != -std::numeric_limits<float>::infinity()) & 
               (depth_map >= 0);

        // マスクを使用して有効な値のみを抽出
        cv::Scalar mean_depth = cv::mean(depth_map, mask);
        //std::cout << "Mean depth: " << mean_depth[0] << std::endl;

        std_msgs::msg::Float32 mean_depth_msg;

        mean_depth_msg.data = static_cast<float>(mean_depth[0]);
        mean_depth_pub_->publish(mean_depth_msg);

        // Manually scale the depth values to 0-255 range
        float min_fix = 0.01f; // Minimum depth value you are interested in (meters)
        float max_fix = 2.0f; // Maximum depth value you are interested in (meters)
        cv::Mat scaled_depth_map = (depth_map - min_fix) * (255.0 / (max_fix - min_fix));
        scaled_depth_map = cv::min(cv::max(scaled_depth_map, 0.0f), 255.0f); // Clipping to 0-255

        // Convert to 8-bit image
        scaled_depth_map.convertTo(scaled_depth_map, CV_8U);

        // Convert grayscale depth map to a ROS message and publish
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", scaled_depth_map).toImageMsg();
        disparity_pub_->publish(*depth_msg);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mean_depth_pub_;
        cv::Mat left_image_, right_image_;
        cv::Ptr<cv::StereoBM> stereo_bm_;

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
    rclcpp::spin(std::make_shared<StereoMatchingNode>());
    rclcpp::shutdown();
    return 0;
}