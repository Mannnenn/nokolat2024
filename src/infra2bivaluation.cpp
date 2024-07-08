#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor() : Node("image_processor")
  {
	// サブスクライバーの設定
	subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
	  "/camera/camera/infra1/image_rect_raw", 10,
	  std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

	// パブリッシャーの設定
	publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/camera/infra1/bivaluation", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
	// ROSメッセージをOpenCV形式に変換
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
	  RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	  return;
	}

	// OpenCVを使用してTHRESH_OTSUを適用
	cv::Mat thresholded;
	cv::threshold(cv_ptr->image, thresholded, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	// 変換された画像をROSメッセージに変換
	cv_bridge::CvImage out_msg;
	out_msg.header = msg->header; // タイムスタンプとフレームIDをコピー
	out_msg.encoding = sensor_msgs::image_encodings::MONO8; // エンコーディングを設定
	out_msg.image = thresholded; // 画像データを設定

	// パブリッシュ
	publisher_->publish(*out_msg.toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}