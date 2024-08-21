#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageDifferenceNode : public rclcpp::Node
{
public:
  ImageDifferenceNode() : Node("image_difference_node")
  {
    // パラメータの宣言
    this->declare_parameter<std::string>("input_ir_image_topic_name", "/camera/camera/infra/image_rect_raw");
    this->declare_parameter<std::string>("output_image_topic_name", "/camera/camera/infra/diff");

    // パラメータの取得
    std::string input_ir_image_topic_name;
    this->get_parameter("input_ir_image_topic_name", input_ir_image_topic_name);
    std::string output_image_topic_name;
    this->get_parameter("output_image_topic_name", output_image_topic_name);

    // サブスクライバーの初期化
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_ir_image_topic_name, 10,
        std::bind(&ImageDifferenceNode::topic_callback, this, std::placeholders::_1));

    // パブリッシャーの初期化
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_name, 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat current_frame = cv_ptr->image;

    if (previous_frame_.empty())
    {
      previous_frame_ = current_frame.clone();
      previous_frame_.convertTo(previous_frame_, CV_32F); // float型に変換
      return;
    }

    // 移動平均を更新
    cv::accumulateWeighted(current_frame, previous_frame_, 0.01);

    // 現在のフレームと移動平均との差を計算
    cv::Mat previous_frame_abs;
    cv::convertScaleAbs(previous_frame_, previous_frame_abs);
    cv::Mat diff;
    cv::absdiff(current_frame, previous_frame_abs, diff);

    // 差分画像をパブリッシュ
    auto diff_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", diff).toImageMsg();
    publisher_->publish(*diff_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::Mat previous_frame_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageDifferenceNode>());
  rclcpp::shutdown();
  return 0;
}