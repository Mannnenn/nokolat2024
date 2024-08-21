#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor() : Node("image_processor")
  {
    // パラメータの宣言
    this->declare_parameter<std::string>("input_image_topic_name", "/camera/camera/infra/image_diff");
    this->declare_parameter<std::string>("output_image_topic_name", "/camera/camera/infra/image_thresholded");
    this->declare_parameter<std::string>("output_cog_topic_name", "/cog_x");

    // パラメータの取得
    std::string input_image_topic_name;
    this->get_parameter("input_image_topic_name", input_image_topic_name);
    std::string output_image_topic_name;
    this->get_parameter("output_image_topic_name", output_image_topic_name);
    std::string output_cog_topic_name;
    this->get_parameter("output_cog_topic_name", output_cog_topic_name);

    // サブスクライバーの設定
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_image_topic_name, 10,
        std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

    // パブリッシャーの設定
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_name, 10);
    cog_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(output_cog_topic_name, 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

    // 2値化
    cv::Mat &img = cv_ptr->image;
    cv::threshold(img, img, 80, 255, cv::THRESH_BINARY);

    // 重心のxとyの値をパブリッシュ
    cv::Moments m = cv::moments(img, true);
    float cog_x = m.m10 / m.m00;
    float cog_y = m.m01 / m.m00;

    geometry_msgs::msg::Point cog_msg;
    cog_msg.x = cog_x;
    cog_msg.y = cog_y;
    cog_msg.z = 0;
    cog_publisher_->publish(cog_msg);
    // 変換された画像をROSメッセージに変換してパブリッシュ
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;                           // タイムスタンプとフレームIDをコピー
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // エンコーディングを設定
    out_msg.image = img;                                    // 画像データを設定

    // パブリッシュ
    image_publisher_->publish(*out_msg.toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cog_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}