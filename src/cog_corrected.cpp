#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>

class StereoMatchingNode : public rclcpp::Node
{
public:
    StereoMatchingNode() : Node("stereo_matching_node")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_left_image_topic_name", "/camera/camera/infra1/image_diff");
        this->declare_parameter<std::string>("input_right_image_topic_name", "/camera/camera/infra2/image_diff");
        this->declare_parameter<std::string>("output_left_cog_topic_name", "/cog_left");
        this->declare_parameter<std::string>("output_right_cog_topic_name", "/cog_right");

        // パラメータの取得
        std::string input_left_image_topic_name;
        this->get_parameter("input_left_image_topic_name", input_left_image_topic_name);
        std::string input_right_image_topic_name;
        this->get_parameter("input_right_image_topic_name", input_right_image_topic_name);
        std::string output_left_cog_topic_name;
        this->get_parameter("output_left_cog_topic_name", output_left_cog_topic_name);
        std::string output_right_cog_topic_name;
        this->get_parameter("output_right_cog_topic_name", output_right_cog_topic_name);

        // YAMLファイルのパスを取得する
        this->declare_parameter<std::string>("yaml_file", "/param/camera_param.yaml");
        std::string yaml_file_path;
        this->get_parameter("yaml_file", yaml_file_path);

        // YAMLファイルを読み込む
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        auto camera_info = config["camera_info"];

        // 解像度を取得
        int width = camera_info["width"].as<int>();
        int height = camera_info["height"].as<int>();
        cv::Size image_size(width, height);

        camera_matrix_left = (cv::Mat_<double>(3, 3) << 
            camera_info["K"][0].as<double>(), camera_info["K"][1].as<double>(), camera_info["K"][2].as<double>(),
            camera_info["K"][3].as<double>(), camera_info["K"][4].as<double>(), camera_info["K"][5].as<double>(),
            camera_info["K"][6].as<double>(), camera_info["K"][7].as<double>(), camera_info["K"][8].as<double>()
        );

        dist_coeffs_left = (cv::Mat_<double>(1, 5) << 
            camera_info["D"][0].as<double>(), camera_info["D"][1].as<double>(), camera_info["D"][2].as<double>(), 
            camera_info["D"][3].as<double>(), camera_info["D"][4].as<double>()
        );

        camera_matrix_right = camera_matrix_left.clone();
        dist_coeffs_right = dist_coeffs_left.clone();

        R = (cv::Mat_<double>(3, 3) << 
            camera_info["R"][0].as<double>(), camera_info["R"][1].as<double>(), camera_info["R"][2].as<double>(),
            camera_info["R"][3].as<double>(), camera_info["R"][4].as<double>(), camera_info["R"][5].as<double>(),
            camera_info["R"][6].as<double>(), camera_info["R"][7].as<double>(), camera_info["R"][8].as<double>()
        );

        // YAMLファイルから投影行列Pを読み込む
        auto P = camera_info["P"].as<std::vector<double>>();
        T = (cv::Mat_<double>(3, 1) << P[3], P[7], 0.0);  // 平行移動ベクトルのx成分とy成分を設定


        // ステレオカメラのキャリブレーション
        cv::stereoRectify(camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, 
                          image_size, R, T, R1, R2, P1, P2, Q);

        // 整列用のマップを作成
        cv::initUndistortRectifyMap(camera_matrix_left, dist_coeffs_left, R1, P1, image_size, CV_32FC1, map1_left, map2_left);
        cv::initUndistortRectifyMap(camera_matrix_right, dist_coeffs_right, R2, P2, image_size, CV_32FC1, map1_right, map2_right);

        // 左右の画像を購読する
        left_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            input_left_image_topic_name, 10, std::bind(&StereoMatchingNode::left_image_callback, this, std::placeholders::_1));
        right_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            input_right_image_topic_name, 10, std::bind(&StereoMatchingNode::right_image_callback, this, std::placeholders::_1));

        cog_left_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(output_left_cog_topic_name, 10);
        cog_right_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(output_right_cog_topic_name, 10);
    }

private:
    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        left_image = cv_bridge::toCvCopy(msg, "mono8")->image;
        process_images();
    }

    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        right_image = cv_bridge::toCvCopy(msg, "mono8")->image;
        //process_images();
    }

    void process_images()
    {
        if (!left_image.empty() && !right_image.empty())
        {
            // 画像を整列
            cv::Mat rectified_left, rectified_right;
            cv::remap(left_image, rectified_left, map1_left, map2_left, cv::INTER_LINEAR);
            cv::remap(right_image, rectified_right, map1_right, map2_right, cv::INTER_LINEAR);

            // rectified_left の処理
            process_and_publish_cog(rectified_left, cog_left_publisher_);

            // rectified_right の処理
            process_and_publish_cog(rectified_right, cog_right_publisher_);
        }
    }

    void process_and_publish_cog(const cv::Mat& image, rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher)
    {
        // 2値化
        cv::Mat binary_image;
        cv::threshold(image, binary_image, 80, 255, cv::THRESH_BINARY);

        // 重心の計算
        cv::Moments m = cv::moments(binary_image, true);
        float cog_x = m.m10 / m.m00;
        float cog_y = m.m01 / m.m00;

        // 重心の値をパブリッシュ
        geometry_msgs::msg::Point cog_msg;
        cog_msg.x = cog_x;
        cog_msg.y = cog_y;
        cog_msg.z = 0;
        publisher->publish(cog_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cog_left_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cog_right_publisher_;

    cv::Mat left_image, right_image;
    cv::Mat camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right;
    cv::Mat R, T, R1, R2, P1, P2, Q;
    cv::Mat map1_left, map2_left, map1_right, map2_right;
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, 15);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoMatchingNode>());
    rclcpp::shutdown();
    return 0;
}