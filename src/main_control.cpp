#include <rclcpp/rclcpp.hpp>

#include <unordered_map>
#include <string>
#include <yaml-cpp/yaml.h>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nokolat2024_msg/msg/rpy.hpp"
#include "nokolat2024_msg/msg/command.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_neutral_position_topic_name", "/neutral_position"); // パラメータの宣言
        this->declare_parameter<std::string>("input_mode_topic_name", "/mode");
        this->declare_parameter<std::string>("input_angular_topic_name", "/rpy");
        this->declare_parameter<std::string>("input_altitude_stamped_topic_name", "/altitude");
        this->declare_parameter<std::string>("output_command_topic_name", "/command_send");

        // パラメータの取得
        std::string input_neutral_position_topic_name;
        this->get_parameter("input_neutral_position_topic_name", input_neutral_position_topic_name);
        std::string input_mode_topic_name;
        this->get_parameter("input_mode_topic_name", input_mode_topic_name);
        std::string input_angular_topic_name;
        this->get_parameter("input_angular_topic_name", input_angular_topic_name);
        std::string input_altitude_stamped_topic_name;
        this->get_parameter("input_altitude_stamped_topic_name", input_altitude_stamped_topic_name);
        std::string output_command_topic_name;
        this->get_parameter("output_command_topic_name", output_command_topic_name);

        // SubscriberとPublisherの設定
        neutral_position_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Command>(input_neutral_position_topic_name, 10, std::bind(&MyNode::neutral_position_callback, this, std::placeholders::_1));

        mode_subscriber_ = this->create_subscription<std_msgs::msg::String>(input_mode_topic_name, 10, std::bind(&MyNode::mode_callback, this, std::placeholders::_1));

        rpy_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Rpy>(input_angular_topic_name, 10, std::bind(&MyNode::rpy_callback, this, std::placeholders::_1));

        altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(input_altitude_stamped_topic_name, 10, std::bind(&MyNode::altitude_callback, this, std::placeholders::_1));

        command_publisher_ = this->create_publisher<nokolat2024_msg::msg::Command>(output_command_topic_name, 10);

        // YAMLファイルのパスを取得する
        this->declare_parameter<std::string>("yaml_control_config", "/home/oga/ros2_humble/install/nokolat2024/share/nokolat2024/param/control_param.yaml");
        std::string yaml_control_config_path;
        this->get_parameter("yaml_control_config", yaml_control_config_path);

        try
        {
            // YAMLファイルを読み込む
            YAML::Node control_config_ = YAML::LoadFile(yaml_control_config_path);
            if (!control_config_["control_info_config"])
            {
                throw std::runtime_error("control_info_config not found in " + yaml_control_config_path);
            }
            control_info_config_ = control_config_["control_info_config"];
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

        // パラメータを取得
        // 各動作上限,下限値を取得
        config.throttle_max = control_info_config_["chassis"]["throttle"]["max"].as<int>();
        config.throttle_min = control_info_config_["chassis"]["throttle"]["min"].as<int>();
        config.elevator_max = control_info_config_["chassis"]["elevator"]["max"].as<int>();
        config.elevator_min = control_info_config_["chassis"]["elevator"]["min"].as<int>();
        config.aileron_max_r = control_info_config_["chassis"]["aileron"]["right"]["max"].as<int>();
        config.aileron_min_r = control_info_config_["chassis"]["aileron"]["right"]["min"].as<int>();
        config.aileron_max_l = control_info_config_["chassis"]["aileron"]["left"]["max"].as<int>();
        config.aileron_min_l = control_info_config_["chassis"]["aileron"]["left"]["min"].as<int>();
        config.rudder_max = control_info_config_["chassis"]["rudder"]["max"].as<int>();
        config.rudder_min = control_info_config_["chassis"]["rudder"]["min"].as<int>();
        config.drop_max = control_info_config_["chassis"]["drop"]["max"].as<int>();
        config.drop_center = control_info_config_["chassis"]["drop"]["center"].as<int>();
        config.drop_min = control_info_config_["chassis"]["drop"]["min"].as<int>();

        RCLCPP_INFO(this->get_logger(), "get chassis parameter");

        // ゲインなどのパラメータを取得
        gain.throttle_gain = control_info_config_["auto_turning"]["gain"]["throttle"]["p"].as<double>();
        gain.elevator_gain = control_info_config_["auto_turning"]["gain"]["elevator"]["p"].as<double>();
        gain.aileron_gain = control_info_config_["auto_turning"]["gain"]["aileron"]["p"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get gain parameter");

        // 制御目標値を取得
        target.velocity_target = control_info_config_["auto_turning"]["target"]["velocity"].as<double>();
        target.altitude_target = control_info_config_["auto_turning"]["target"]["altitude"].as<double>();
        target.roll_target = control_info_config_["auto_turning"]["target"]["roll"].as<double>();
        target.pitch_target = control_info_config_["auto_turning"]["target"]["pitch"].as<double>();
        target.rudder_target = control_info_config_["auto_turning"]["target"]["rudder"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get target parameter");
    }

private:
    void neutral_position_callback(const nokolat2024_msg::msg::Command::SharedPtr msg)
    {
        neutral_position_.throttle = msg->throttle;
        neutral_position_.elevator = msg->elevator;
        neutral_position_.aileron_r = msg->aileron_r;
        neutral_position_.aileron_l = msg->aileron_l;
        neutral_position_.rudder = msg->rudder;
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        control_mode_ = msg->data;
    }

    void rpy_callback(const nokolat2024_msg::msg::Rpy::SharedPtr msg)

    {
        pose_received_.roll = msg->roll;
        pose_received_.pitch = msg->pitch;

        if (control_mode_ == control_mode_map.at(CONTROL_MODE::AUTO_TURNING))
        {
            auto_turning_control();
            // RCLCPP_INFO(this->get_logger(), "AUTO_TURNING");
        }
    }

    void altitude_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        pose_received_.z = msg->point.z;
    }

    void auto_turning_control()
    {
        // 制御値を計算
        double throttle = neutral_position_.throttle + gain.throttle_gain * (target.velocity_target - pose_received_.z);
        double elevator = neutral_position_.elevator + gain.elevator_gain * (target.altitude_target - pose_received_.z);
        double aileron_r = neutral_position_.aileron_r + gain.aileron_gain * (target.roll_target - pose_received_.roll);
        double aileron_l = neutral_position_.aileron_l + gain.aileron_gain * (target.roll_target - pose_received_.roll);
        double rudder = neutral_position_.rudder; //+ gain.aileron_gain * (target.rudder_target - pose_received_.roll);

        // 制御値を送信
        nokolat2024_msg::msg::Command command;
        command.throttle = throttle;
        command.elevator = elevator;
        command.aileron_r = aileron_r;
        command.aileron_l = aileron_l;
        command.rudder = rudder;
        command.dropping_device = neutral_position_.drop;
        command_publisher_->publish(command);
    }

    // Define control mode
    enum CONTROL_MODE
    {
        MANUAL = 0,
        AUTO_TURNING = 1,
        AUTO_RISE_TURNING = 2,
        AUTO_LANDING = 3,
        AUTO_EIGHT = 4,
    };

    // マップを初期化
    const std::unordered_map<int16_t, std::string> control_mode_map = {
        {CONTROL_MODE::MANUAL, "MANUAL"},
        {CONTROL_MODE::AUTO_TURNING, "AUTO_TURNING"},
        {CONTROL_MODE::AUTO_RISE_TURNING, "AUTO_RISE_TURNING"},
        {CONTROL_MODE::AUTO_LANDING, "AUTO_LANDING"},
        {CONTROL_MODE::AUTO_EIGHT, "AUTO_EIGHT"}};

    struct ControlInfo_config
    {
        int throttle_max;
        int throttle_min;
        int elevator_max;
        int elevator_min;
        int aileron_max_r;
        int aileron_min_r;
        int aileron_max_l;
        int aileron_min_l;
        int rudder_max;
        int rudder_min;
        int drop_max;
        int drop_center;
        int drop_min;
    };

    struct ControlInfo_gain
    {
        double throttle_gain;
        double elevator_gain;
        double aileron_gain;
    };

    struct ControlInfo_target
    {
        double velocity_target;
        double altitude_target;
        double roll_target;
        double pitch_target;
        double rudder_target;
    };

    struct Command
    {
        double throttle;
        double elevator;
        double aileron_r;
        double aileron_l;
        double rudder;
        double drop;
    };

    struct Pose
    {
        double roll;
        double pitch;
        double z;
    };

    // 制御情報を格納
    ControlInfo_config config;
    ControlInfo_gain gain;
    ControlInfo_target target;

    Pose pose_received_;
    Command neutral_position_;

    // パラメータを取得
    YAML::Node control_info_config_;

    std::string control_mode_;

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr neutral_position_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitude_subscriber_;

    rclcpp::Publisher<nokolat2024_msg::msg::Command>::SharedPtr command_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}