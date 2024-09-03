#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <deque>

#include <Eigen/Dense>

#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <yaml-cpp/yaml.h>

class positionEstimateNode : public rclcpp::Node
{
public:
    positionEstimateNode() : Node("stereo_matching_node"), last_time_(this->now())
    {
        // パラメータの宣言
        // パラメータの宣言
        this->declare_parameter<std::string>("input_left_cog_topic_name", "/left/cog_raw");
        this->declare_parameter<std::string>("input_right_cog_topic_name", "/right/cog_raw");
        this->declare_parameter<std::string>("input_depth_topic_name", "/depth");
        this->declare_parameter<std::string>("output_position_topic_name", "/position");

        // パラメータの取得
        std::string input_left_cog_topic_name;
        this->get_parameter("input_left_cog_topic_name", input_left_cog_topic_name);
        std::string input_right_cog_topic_name;
        this->get_parameter("input_right_cog_topic_name", input_right_cog_topic_name);
        std::string input_depth_topic_name;
        this->get_parameter("input_depth_topic_name", input_depth_topic_name);
        std::string output_position_topic_name;
        this->get_parameter("output_position_topic_name", output_position_topic_name);

        left_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            input_left_cog_topic_name, 10, std::bind(&positionEstimateNode::leftCogCallback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            input_right_cog_topic_name, 10, std::bind(&positionEstimateNode::rightCogCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(input_depth_topic_name, 10, std::bind(&positionEstimateNode::depthCallback, this, std::placeholders::_1));

        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(output_position_topic_name, 10);

        // YAMLファイルのパスを取得する
        this->declare_parameter<std::string>("yaml_right_file", "/param/camera_param_right.yaml");
        std::string yaml_right_file_path;
        this->get_parameter("yaml_right_file", yaml_right_file_path);

        try
        {
            // YAMLファイルを読み込む
            YAML::Node config_right = YAML::LoadFile(yaml_right_file_path);
            if (!config_right["camera_info_right"])
            {
                throw std::runtime_error("camera_info_right not found in " + yaml_right_file_path);
            }
            camera_info_right = config_right["camera_info_right"];
        }
        catch (const YAML::BadFile &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
            return;
        }
        catch (const std::runtime_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error: %s", e.what());
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            return;
        }

        focal_length = camera_info_right["K"][0].as<double>();

        center_x = camera_info_right["K"][2].as<double>();
        center_y = camera_info_right["K"][5].as<double>();

        // カルマンフィルタの初期化
        x_ = Eigen::VectorXd::Zero(6);                          // 状態ベクトル [x, y, z, vx, vy, vz]
        P_ = Eigen::MatrixXd::Identity(6, 6);                   // 共分散行列
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;             // プロセスノイズ
        R_ = Eigen::MatrixXd::Identity(3, 3) * 1;               // 観測ノイズ
        A_ = Eigen::MatrixXd::Identity(6, 6);                   // 状態遷移行列
        H_ = Eigen::MatrixXd::Zero(3, 6);                       // 観測モデル
        H_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3); // 位置の部分だけを観測
    }

private:
    void leftCogCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        position_left_ = *msg;
    }

    void rightCogCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        position_right_ = *msg;
    }

    void depthCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        depth_ = msg->data;
        processPosition();
    }

    void processPosition()
    {
        if (std::isnan(depth_))
        {
            RCLCPP_WARN(this->get_logger(), "left_cog or right_cog is NaN");
            return;
        }

        float mean_x = (position_left_.x + position_right_.x) / 2;
        float mean_y = (position_left_.y + position_right_.y) / 2;

        // calc position,In Realsense D455, the depth is the distance from the camera to the object.

        float raw_y = (mean_x - center_x) * depth_ / focal_length;
        float raw_z = (mean_y - center_y) * depth_ / focal_length;
        float raw_x = depth_;

        float dt = (this->now() - last_time_).seconds();
        // 状態遷移行列 A に時間差分を反映
        A_(0, 3) = dt;
        A_(1, 4) = dt;
        A_(2, 5) = dt;

        // ステレオカメラからの観測値を取得 (ここでは例として単一のポイントを使用)
        Eigen::VectorXd z(3);
        z << raw_x, raw_y, raw_z;

        // カルマンフィルタ予測ステップ
        x_ = A_ * x_;
        P_ = A_ * P_ * A_.transpose() + Q_;

        // カルマンフィルタ更新ステップ
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * (z - H_ * x_);
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;

        position_.x = x_(0);
        position_.y = x_(1);
        position_.z = x_(2);

        // printf("x: %f, y: %f\n", position_.x, position_.z);

        position_pub_->publish(position_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr left_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;

    geometry_msgs::msg::Point position_left_;
    geometry_msgs::msg::Point position_right_;

    YAML::Node camera_info_right;

    geometry_msgs::msg::Point position_;

    float focal_length;
    float center_x;
    float center_y;

    float depth_;

    rclcpp::Time last_time_;
    Eigen::VectorXd x_; // 状態ベクトル [x, y, z, vx, vy, vz]
    Eigen::MatrixXd P_; // 共分散行列
    Eigen::MatrixXd Q_; // プロセスノイズ
    Eigen::MatrixXd R_; // 観測ノイズ
    Eigen::MatrixXd A_; // 状態遷移行列
    Eigen::MatrixXd H_; // 観測モデル
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<positionEstimateNode>());
    rclcpp::shutdown();
    return 0;
}
