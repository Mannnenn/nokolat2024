#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class StereoMatchingNode : public rclcpp::Node
{
public:
  StereoMatchingNode() : Node("stereo_matching_node")
  {
        // パラメータの宣言
    this->declare_parameter<std::string>("input_left_image_topic_name", "/left/image_raw");
    this->declare_parameter<std::string>("input_right_image_topic_name", "/right/image_raw");
    this->declare_parameter<std::string>("output_image_topic_name", "/camera/camera/infra/depth_image");

    // パラメータの取得
    std::string input_left_image_topic_name;
    this->get_parameter("input_left_image_topic_name", input_left_image_topic_name);
    std::string input_right_image_topic_name;
    this->get_parameter("input_right_image_topic_name", input_right_image_topic_name);
    std::string output_image_topic_name;
    this->get_parameter("output_image_topic_name", output_image_topic_name);


    left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_left_image_topic_name, 10, std::bind(&StereoMatchingNode::leftImageCallback, this, std::placeholders::_1));

    right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_right_image_topic_name, 10, std::bind(&StereoMatchingNode::rightImageCallback, this, std::placeholders::_1));

    disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_name, 10);

    // Create StereoBM object
    stereo_bm_ = cv::StereoBM::create();
  }

private:
  void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    left_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    processImages();
  }

  void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    right_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    processImages();
  }

  void processImages()
  {
    if (!left_image_.empty() && !right_image_.empty())
    {
        // パラメータの調整
        stereo_bm_->setBlockSize(15); // ブロックサイズを15に設定
        stereo_bm_->setNumDisparities(64); // 視差の範囲を128に設定
        stereo_bm_->setPreFilterType(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE); // プリフィルタのタイプを設定
        stereo_bm_->setTextureThreshold(1); // テクスチャの閾値を10に設定

      cv::Mat disparity;
      stereo_bm_->compute(left_image_, right_image_, disparity);



    // Calculate depth map from disparity
    cv::Mat depth_map;
    float focal_length = 640.0f; // Set your camera's focal length in pixels
    float baseline = 10.7f; // Set the distance between your cameras in meters
    //Cast disparity to float
    disparity.convertTo(disparity, CV_32F);

    depth_map = focal_length * baseline / disparity;



    //Get min and max depth values
    double min_depth, max_depth;
    cv::minMaxLoc(depth_map, &min_depth, &max_depth);
    // Print min and max depth values
    RCLCPP_INFO(this->get_logger(), "Min depth: %f, Max depth: %f", min_depth, max_depth);

    // Manually scale the depth values to 0-255 range
    float min_fix = 0.01f; // Minimum depth value you are interested in (meters)
    float max_fix = 20.0f; // Maximum depth value you are interested in (meters)
    cv::Mat scaled_depth_map = (depth_map - min_fix) * (255.0 / (max_fix - min_fix));
    scaled_depth_map = cv::min(cv::max(scaled_depth_map, 0.0f), 255.0f); // Clipping to 0-255

    // Convert to 8-bit image
    scaled_depth_map.convertTo(scaled_depth_map, CV_8U);

    // Apply colormap
    cv::Mat colored_depth_map;
    cv::applyColorMap(scaled_depth_map, colored_depth_map, cv::COLORMAP_JET);

    // Convert colored depth map to a ROS message and publish
    auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colored_depth_map).toImageMsg();
    disparity_pub_->publish(*depth_msg);

    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
  cv::Mat left_image_, right_image_;
  cv::Ptr<cv::StereoBM> stereo_bm_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoMatchingNode>());
  rclcpp::shutdown();
  return 0;
}