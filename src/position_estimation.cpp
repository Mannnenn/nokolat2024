#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>

class StereoMatchingNode : public rclcpp::Node {
public:
        StereoMatchingNode() : Node("stereo_matching_node") {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_image_topic_name", "/left/image_raw");
        this->declare_parameter<std::string>("input_depth", "/depth");
        this->declare_parameter<std::string>("output_position_topic_name", "/camera/camera/infra/depth_image");

        // パラメータの取得
        std::string input_image_topic_name;
        this->get_parameter("input_image_topic_name", input_image_topic_name);
        std::string input_depth;
        this->get_parameter("input_depth", input_depth);
        std::string output_position_topic_name;
        this->get_parameter("output_position_topic_name", output_position_topic_name);


        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_image_topic_name, 10, std::bind(&StereoMatchingNode::imageCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        input_depth, 10, std::bind(&StereoMatchingNode::depthCallback, this, std::placeholders::_1));

        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(output_position_topic_name, 10);

    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
        processImages();
    }

    void depthCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        depth_ = msg->data;
    }

    void processImages() {
        if (image_.empty()) {
            return;
        }

        cv::Mat image = image_;

        // 2値化
        cv::Mat binary_image;
        cv::threshold(image, binary_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // calc center
        cv::Moments mu = cv::moments(binary_image, false);
        float x = mu.m10 / mu.m00;
        float y = mu.m01 / mu.m00;


        // calc position,In Realsense D455, the depth is the distance from the camera to the object.

        float focal_length = 640.0f;
        float center_x = 424;
        float center_y = 240;


        position_.y = (x - center_x) * depth_ / focal_length;//left as y axis.
        position_.z = (y - center_y) * depth_ / focal_length; //up as z axis.
        position_.x = depth_; //forward as x axis.

        //printf("x: %f, y: %f\n", position_.x, position_.z);

        position_pub_->publish(position_);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
        cv::Mat image_;
        float depth_;
        geometry_msgs::msg::Point position_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoMatchingNode>());
    rclcpp::shutdown();
    return 0;
}