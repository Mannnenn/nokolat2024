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
    // パラメータの宣言
    this->declare_parameter<std::string>("input_image_topic_name", "/camera/camera/infra/image_diff");
    this->declare_parameter<std::string>("output_image_topic_name", "/camera/camera/infra/image_thresholded");

    // パラメータの取得
    std::string input_image_topic_name;
    this->get_parameter("input_image_topic_name", input_image_topic_name);
    std::string output_image_topic_name;
    this->get_parameter("output_image_topic_name", output_image_topic_name);


	// サブスクライバーの設定
	subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
	  input_image_topic_name, 10,
	  std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

	// パブリッシャーの設定
	publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_name, 10);
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

  // 低輝度の点をフィルタリング
  cv::Mat& img = cv_ptr->image;
  for (int y = 0; y < img.rows; y++)
  {
    for (int x = 0; x < img.cols; x++)
    {
      // 輝度値が50未満の場合、ピクセルを黒に設定
      if (img.at<uchar>(y, x) < 20)
      {
        img.at<uchar>(y, x) = 0;
      }
    }
  }

  // 変換された画像をROSメッセージに変換してパブリッシュ
  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header; // タイムスタンプとフレームIDをコピー
  out_msg.encoding = sensor_msgs::image_encodings::MONO8; // エンコーディングを設定
  out_msg.image = img; // 画像データを設定

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