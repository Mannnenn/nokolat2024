#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

// Define the type for the spline
typedef Eigen::Spline<double, 3> Spline3d;

class PathGenerator : public rclcpp::Node
{
public:
    PathGenerator() : Node("path_generator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathGenerator::on_timer, this));
    }

private:
    void on_timer()
    {
        try
        {
            auto transform_map_to_start = tf_buffer_.lookupTransform("map", "start", tf2::TimePointZero);
            auto transform_start_to_waypoint1 = tf_buffer_.lookupTransform("map", "waypoint_1", tf2::TimePointZero);
            auto transform_waypoint1_to_waypoint2 = tf_buffer_.lookupTransform("map", "waypoint_2", tf2::TimePointZero);
            auto transform_waypoint2_to_waypoint3 = tf_buffer_.lookupTransform("map", "waypoint_3", tf2::TimePointZero);
            auto transform_waypoint3_to_goal = tf_buffer_.lookupTransform("map", "goal", tf2::TimePointZero);

            // Define control points and orientations
            // ROS2::xyzw, Eigen::Quaterniond::wxyz
            std::vector<Eigen::Vector3d> control_points = {
                {transform_map_to_start.transform.translation.x,
                 transform_map_to_start.transform.translation.y,
                 transform_map_to_start.transform.translation.z},
                {transform_start_to_waypoint1.transform.translation.x,
                 transform_start_to_waypoint1.transform.translation.y,
                 transform_start_to_waypoint1.transform.translation.z},
                {transform_waypoint1_to_waypoint2.transform.translation.x,
                 transform_waypoint1_to_waypoint2.transform.translation.y,
                 transform_waypoint1_to_waypoint2.transform.translation.z},
                {transform_waypoint2_to_waypoint3.transform.translation.x,
                 transform_waypoint2_to_waypoint3.transform.translation.y,
                 transform_waypoint2_to_waypoint3.transform.translation.z},
                {transform_waypoint3_to_goal.transform.translation.x,
                 transform_waypoint3_to_goal.transform.translation.y,
                 transform_waypoint3_to_goal.transform.translation.z}};

            std::vector<Eigen::Quaterniond> orientations = {
                Eigen::Quaterniond(
                    transform_map_to_start.transform.rotation.w,
                    transform_map_to_start.transform.rotation.x,
                    transform_map_to_start.transform.rotation.y,
                    transform_map_to_start.transform.rotation.z),
                Eigen::Quaterniond(
                    transform_start_to_waypoint1.transform.rotation.w,
                    transform_start_to_waypoint1.transform.rotation.x,
                    transform_start_to_waypoint1.transform.rotation.y,
                    transform_start_to_waypoint1.transform.rotation.z),
                Eigen::Quaterniond(
                    transform_waypoint1_to_waypoint2.transform.rotation.w,
                    transform_waypoint1_to_waypoint2.transform.rotation.x,
                    transform_waypoint1_to_waypoint2.transform.rotation.y,
                    transform_waypoint1_to_waypoint2.transform.rotation.z),
                Eigen::Quaterniond(
                    transform_waypoint2_to_waypoint3.transform.rotation.w,
                    transform_waypoint2_to_waypoint3.transform.rotation.x,
                    transform_waypoint2_to_waypoint3.transform.rotation.y,
                    transform_waypoint2_to_waypoint3.transform.rotation.z),
                Eigen::Quaterniond(
                    transform_waypoint3_to_goal.transform.rotation.w,
                    transform_waypoint3_to_goal.transform.rotation.x,
                    transform_waypoint3_to_goal.transform.rotation.y,
                    transform_waypoint3_to_goal.transform.rotation.z)};

            // Create the spline
            Eigen::MatrixXd points(3, control_points.size());
            for (size_t i = 0; i < control_points.size(); ++i)
            {
                points.col(i) = control_points[i];
            }

            Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 2); // 2nd order spline

            // Generate the path
            geometry_msgs::msg::PoseArray path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "map";

            for (double t = 0; t <= 1; t += 0.01)
            {
                Eigen::Vector3d point = spline(t);
                Eigen::Quaterniond orientation = interpolate_orientation(orientations, t);

                orientation.inverse().normalize();
                geometry_msgs::msg::Pose pose;
                pose.position.x = point.x();
                pose.position.y = point.y();
                pose.position.z = point.z();
                pose.orientation.x = orientation.x();
                pose.orientation.y = orientation.y();
                pose.orientation.z = orientation.z();
                pose.orientation.w = orientation.w();
                path_msg.poses.push_back(pose);
            }

            publisher_->publish(path_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    Eigen::Quaterniond interpolate_orientation(const std::vector<Eigen::Quaterniond> &orientations, double t)
    {
        // Simple linear interpolation for orientations
        size_t n = orientations.size();
        double segment_length = 1.0 / (n - 1);
        size_t segment = std::min(static_cast<size_t>(t / segment_length), n - 2);
        double local_t = (t - segment * segment_length) / segment_length;
        return orientations[segment].slerp(local_t, orientations[segment + 1]);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathGenerator>());
    rclcpp::shutdown();
    return 0;
}