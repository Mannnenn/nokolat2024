#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

class ImageSyncNode : public rclcpp::Node
{
public:
    ImageSyncNode()
        : Node("image_sync_node"),
          sync_(approximate_policy(10), image_sub1_, image_sub2_)

    {
        // サブスクライバーの初期化
        image_sub1_.subscribe(this, "image_topic1");
        image_sub2_.subscribe(this, "image_topic2");

        // タイムシンクロナイザーの初期化
        sync_.registerCallback(&ImageSyncNode::callback, this);

        // パブリッシャーの初期化
        pub1_ = this->create_publisher<sensor_msgs::msg::Image>("synced_image_topic1", 10);
        pub2_ = this->create_publisher<sensor_msgs::msg::Image>("synced_image_topic2", 10);
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr image1, const sensor_msgs::msg::Image::SharedPtr image2)
    {
        // タイムスタンプを揃えた画像をパブリッシュ

        sensor_msgs::msg::Image sync_image = *image2;

        sync_image.header.stamp = image1->header.stamp;

        pub1_->publish(*image1);
        pub2_->publish(sync_image);
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub1_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub2_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
    message_filters::Synchronizer<approximate_policy> sync_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSyncNode>());
    rclcpp::shutdown();
    return 0;
}
