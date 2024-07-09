#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <omp.h>

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
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat current_frame = cv_ptr->image;

    if (!previous_frame_.empty())
    {
      cv::Mat diff = cv::Mat::zeros(current_frame.size(), current_frame.type());

      // OpenMPを使用した並列化
#pragma omp parallel for
      for (int i = 0; i < current_frame.rows; ++i)
      {
        for (int j = 0; j < current_frame.cols; ++j)
        {
          diff.at<uchar>(i, j) = std::abs(current_frame.at<uchar>(i, j) - previous_frame_.at<uchar>(i, j));
        }
      }


      // 差分画像をパブリッシュ
      auto diff_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", diff).toImageMsg();
      publisher_->publish(*diff_msg);
    }

    previous_frame_ = current_frame;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::Mat previous_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageDifferenceNode>());
  rclcpp::shutdown();
  return 0;
}