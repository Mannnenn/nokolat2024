#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_position_topic_name", "/position");
        this->declare_parameter<std::string>("input_polygon_topic_name", "/polygon");
        this->declare_parameter<std::string>("output_position_stamped_topic_name", "/path");

        // パラメータの取得
        std::string input_position_topic_name;
        this->get_parameter("input_position_topic_name", input_position_topic_name);
        std::string input_polygon_topic_name;
        this->get_parameter("input_polygon_topic_name", input_polygon_topic_name);
        std::string output_position_stamped_topic_name;
        this->get_parameter("output_position_stamped_topic_name", output_position_stamped_topic_name);

        subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            input_position_topic_name, 10, std::bind(&PathPublisher::point_callback, this, std::placeholders::_1));
        subscriber_polygon_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            input_polygon_topic_name + "fov3", 10, std::bind(&PathPublisher::polygon_callback, this, std::placeholders::_1)); // 0はindexのデフォルト値
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_position_stamped_topic_name, 10);
    }

private:
    void point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (is_within_pyramid(*msg))
        {
            geometry_msgs::msg::PointStamped pose;
            pose.header.frame_id = "camera_link";
            pose.header.stamp = this->get_clock()->now();
            pose.point = *msg;
            publisher_->publish(pose);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Point is outside the pyramid");
            RCLCPP_INFO(this->get_logger(), "Point: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
            RCLCPP_INFO(this->get_logger(), "Pyramid vertices:");
            for (int i = 0; i < 5; i++)
            {
                RCLCPP_INFO(this->get_logger(), "Vertex %d: [%.2f, %.2f, %.2f]", i, pyramid_vertices_[i].x, pyramid_vertices_[i].y, pyramid_vertices_[i].z);
            }
        }
    }

    void polygon_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {
        // 4角錐の頂点を求める。側面のポリゴンからすべての頂点を求める。頂点は底面の中心の真上にある
        pyramid_vertices_[0].x = msg->polygon.points[0].z;
        pyramid_vertices_[0].y = msg->polygon.points[0].x;
        pyramid_vertices_[0].z = msg->polygon.points[0].y;

        pyramid_vertices_[1].x = msg->polygon.points[1].z;
        pyramid_vertices_[1].y = -msg->polygon.points[1].x;
        pyramid_vertices_[1].z = -msg->polygon.points[1].y;

        pyramid_vertices_[2].x = msg->polygon.points[1].z;
        pyramid_vertices_[2].y = msg->polygon.points[1].x;
        pyramid_vertices_[2].z = -msg->polygon.points[1].y;

        pyramid_vertices_[3].x = msg->polygon.points[1].z;
        pyramid_vertices_[3].y = msg->polygon.points[1].x;
        pyramid_vertices_[3].z = msg->polygon.points[1].y;

        pyramid_vertices_[4].x = msg->polygon.points[1].z;
        pyramid_vertices_[4].y = -msg->polygon.points[1].x;
        pyramid_vertices_[4].z = msg->polygon.points[1].y;
    }

    bool is_within_pyramid(const geometry_msgs::msg::Point &point)
    {
        // 四角錐の頂点
        const auto &p0 = pyramid_vertices_[0];
        const auto &p1 = pyramid_vertices_[1];
        const auto &p2 = pyramid_vertices_[2];
        const auto &p3 = pyramid_vertices_[3];
        const auto &p4 = pyramid_vertices_[4];

        // ベクトル計算のためのヘルパー関数
        auto vector_subtract = [](const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
        {
            geometry_msgs::msg::Point result;
            result.x = a.x - b.x;
            result.y = a.y - b.y;
            result.z = a.z - b.z;
            return result;
        };

        auto dot_product = [](const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        };

        auto cross_product = [](const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
        {
            geometry_msgs::msg::Point result;
            result.x = a.y * b.z - a.z * b.y;
            result.y = a.z * b.x - a.x * b.z;
            result.z = a.x * b.y - a.y * b.x;
            return result;
        };

        // 四角錐の各面の法線ベクトルを計算
        auto normal1 = cross_product(vector_subtract(p1, p0), vector_subtract(p2, p0));
        auto normal2 = cross_product(vector_subtract(p2, p0), vector_subtract(p3, p0));
        auto normal3 = cross_product(vector_subtract(p3, p0), vector_subtract(p4, p0));
        auto normal4 = cross_product(vector_subtract(p4, p0), vector_subtract(p1, p0));
        auto normal_base = cross_product(vector_subtract(p2, p1), vector_subtract(p3, p1));

        // 点が各面の内側にあるかをチェック
        auto check_side = [&](const geometry_msgs::msg::Point &normal, const geometry_msgs::msg::Point &vertex)
        {
            return dot_product(normal, vector_subtract(point, vertex)) >= 0;
        };

        return check_side(normal1, p0) && check_side(normal2, p0) && check_side(normal3, p0) && check_side(normal4, p0) && check_side(normal_base, p1);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscriber_polygon_;
    geometry_msgs::msg::Point pyramid_vertices_[5];
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}