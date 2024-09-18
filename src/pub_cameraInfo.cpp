#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <cmath>

class CameraInfoSubscriber : public rclcpp::Node
{
public:
    CameraInfoSubscriber() : Node("camera_info_subscriber")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("camera_view_area_topic_name", "/camera/camera/infra1/camera_view_area");
        this->declare_parameter<std::string>("camera_info_topic_name", "/camera/camera/infra1/camera_info");

        // パラメータからトピック名を取得
        std::string camera_view_area_topic_name;
        this->get_parameter("camera_view_area_topic_name", camera_view_area_topic_name);
        std::string camera_info_topic_name;
        this->get_parameter("camera_info_topic_name", camera_info_topic_name);

        pub1_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(camera_view_area_topic_name + "camera_fov_triangle1", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(camera_view_area_topic_name + "camera_fov_triangle2", 10);
        pub3_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(camera_view_area_topic_name + "camera_fov_triangle3", 10);
        pub4_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(camera_view_area_topic_name + "camera_fov_triangle4", 10);

        sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_name, 10, std::bind(&CameraInfoSubscriber::cameraInfoCallback, this, std::placeholders::_1));
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // カメラ行列から焦点距離を取得
        double fx = msg->k[0];       // 焦点距離 (x軸)
        double fy = msg->k[4];       // 焦点距離 (y軸)
        double width = msg->width;   // 画像の幅
        double height = msg->height; // 画像の高さ

        // 水平視野角の計算
        double horizontal_fov = 2 * atan(width / (2 * fx));
        // 垂直視野角の計算
        double vertical_fov = 2 * atan(height / (2 * fy));

        // 四角錐の頂点を計算
        double depth = 1.0; // 視野の深さを1メートルと仮定
        double half_width = depth * tan(horizontal_fov / 2);
        double half_height = depth * tan(vertical_fov / 2);

        geometry_msgs::msg::Point32 p0, p1, p2, p3, p4;
        p0.x = 0;
        p0.y = 0;
        p0.z = 0; // カメラの位置
        p1.z = depth;
        p1.x = -half_width;
        p1.y = -half_height;
        p2.z = depth;
        p2.x = half_width;
        p2.y = -half_height;
        p3.z = depth;
        p3.x = half_width;
        p3.y = half_height;
        p4.z = depth;
        p4.x = -half_width;
        p4.y = half_height;

        // 三角形1: p0, p1, p2
        geometry_msgs::msg::PolygonStamped triangle1;
        triangle1.header.stamp = this->now();
        triangle1.header.frame_id = msg->header.frame_id;
        triangle1.polygon.points.push_back(p0);
        triangle1.polygon.points.push_back(p1);
        triangle1.polygon.points.push_back(p2);

        // 三角形2: p0, p2, p3
        geometry_msgs::msg::PolygonStamped triangle2;
        triangle2.header.stamp = this->now();
        triangle2.header.frame_id = msg->header.frame_id;
        triangle2.polygon.points.push_back(p0);
        triangle2.polygon.points.push_back(p2);
        triangle2.polygon.points.push_back(p3);

        // 三角形3: p0, p3, p4
        geometry_msgs::msg::PolygonStamped triangle3;
        triangle3.header.stamp = this->now();
        triangle3.header.frame_id = msg->header.frame_id;
        triangle3.polygon.points.push_back(p0);
        triangle3.polygon.points.push_back(p3);
        triangle3.polygon.points.push_back(p4);

        // 三角形4: p0, p4, p1
        geometry_msgs::msg::PolygonStamped triangle4;
        triangle4.header.stamp = this->now();
        triangle4.header.frame_id = msg->header.frame_id;
        triangle4.polygon.points.push_back(p0);
        triangle4.polygon.points.push_back(p4);
        triangle4.polygon.points.push_back(p1);

        pub1_->publish(triangle1);
        pub2_->publish(triangle2);
        pub3_->publish(triangle3);
        pub4_->publish(triangle4);
    }

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub2_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub3_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub4_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoSubscriber>());
    rclcpp::shutdown();
    return 0;
}