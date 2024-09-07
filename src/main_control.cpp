#include "nokolat2024/main_control.hpp"

class MainControlNode : public rclcpp::Node
{
public:
    MainControlNode() : Node("main_control_node")
    {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_neutral_position_topic_name", "/neutral_position"); // パラメータの宣言
        this->declare_parameter<std::string>("input_command_explicit_topic_name", "/command_explicit");
        this->declare_parameter<std::string>("input_mode_topic_name", "/mode");
        this->declare_parameter<std::string>("input_angular_topic_name", "/rpy");
        this->declare_parameter<std::string>("input_altitude_stamped_topic_name", "/altitude");
        this->declare_parameter<std::string>("output_command_topic_name", "/command_send");

        // パラメータの取得
        std::string input_neutral_position_topic_name;
        this->get_parameter("input_neutral_position_topic_name", input_neutral_position_topic_name);
        std::string input_command_explicit_topic_name;
        this->get_parameter("input_command_explicit_topic_name", input_command_explicit_topic_name);
        std::string input_mode_topic_name;
        this->get_parameter("input_mode_topic_name", input_mode_topic_name);
        std::string input_angular_topic_name;
        this->get_parameter("input_angular_topic_name", input_angular_topic_name);
        std::string input_altitude_stamped_topic_name;
        this->get_parameter("input_altitude_stamped_topic_name", input_altitude_stamped_topic_name);
        std::string output_command_topic_name;
        this->get_parameter("output_command_topic_name", output_command_topic_name);

        // SubscriberとPublisherの設定
        neutral_position_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Command>(input_neutral_position_topic_name, 10, std::bind(&MainControlNode::neutral_position_callback, this, std::placeholders::_1));

        command_explicit_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Command>(input_command_explicit_topic_name, 10, std::bind(&MainControlNode::neutral_position_callback, this, std::placeholders::_1));

        mode_subscriber_ = this->create_subscription<std_msgs::msg::String>(input_mode_topic_name, 10, std::bind(&MainControlNode::mode_callback, this, std::placeholders::_1));

        rpy_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Rpy>(input_angular_topic_name, 10, std::bind(&MainControlNode::rpy_callback, this, std::placeholders::_1));

        altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(input_altitude_stamped_topic_name, 10, std::bind(&MainControlNode::altitude_callback, this, std::placeholders::_1));

        command_publisher_ = this->create_publisher<nokolat2024_msg::msg::Command>(output_command_topic_name, 10);

        // YAMLファイルのパスを取得する
        this->declare_parameter<std::string>("yaml_control_config", "/home/oga/ros2_humble/install/nokolat2024/share/nokolat2024/param/control_param.yaml");
        std::string yaml_control_config_path;
        this->get_parameter("yaml_control_config", yaml_control_config_path);

        // 制御パラメータを読み込む
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
        auto_turning_gain.elevator_gain = control_info_config_["auto_turning"]["gain"]["elevator"]["p"].as<double>();
        auto_turning_gain.aileron_gain = control_info_config_["auto_turning"]["gain"]["aileron"]["p"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get gain parameter");

        // 制御目標値を取得
        auto_turning_target.altitude_target = control_info_config_["auto_turning"]["target"]["altitude"].as<double>();
        auto_turning_target.roll_target = control_info_config_["auto_turning"]["target"]["roll"].as<double>();
        auto_turning_target.throttle_target = control_info_config_["auto_turning"]["target"]["throttle"].as<double>();
        auto_turning_target.rudder_target = control_info_config_["auto_turning"]["target"]["rudder"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get target parameter");

        auto_turning_delay.delay_rudder = control_info_config_["auto_turning"]["delay"]["rudder"].as<uint>();

        RCLCPP_INFO(this->get_logger(), "get delay window parameter");

        last_mode = nokolat2024::main_control::MANUAL;
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

    void command_explicit_callback(const nokolat2024_msg::msg::Command::SharedPtr msg)
    {
        throttle_history_.push_back(msg->throttle);
        if (throttle_history_.size() > 10)
        {
            throttle_history_.pop_front();
        }
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        control_mode_ = msg->data;

        if (last_mode != control_mode_)
        {
            mode_change_flag = true;
        }
        else
        {
            mode_change_flag = false;
        }
        last_mode = control_mode_;
    }

    void rpy_callback(const nokolat2024_msg::msg::Rpy::SharedPtr msg)

    {
        pose_received_.roll = msg->roll;
        pose_received_.pitch = msg->pitch;
        pose_received_.yaw = msg->yaw;

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_TURNING))
        {
            if (mode_change_flag)
            {
                get_target_altitude();
            }
            auto_turning_control();
            RCLCPP_INFO(this->get_logger(), "target_altitude[%f]", auto_turning_target.altitude_target);
            // RCLCPP_INFO(this->get_logger(), "AUTO_TURNING");
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_EIGHT))
        {
            auto_eight_control();
            // RCLCPP_INFO(this->get_logger(), "AUTO_EIGHT");
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_RISE_TURNING))
        {
            // auto_rise_turning_control();
            //  RCLCPP_INFO(this->get_logger(), "AUTO_RISE_TURNING");
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_LANDING))
        {
            // auto_landing_control();
            //  RCLCPP_INFO(this->get_logger(), "AUTO_LANDING");
        }
    }

    void altitude_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        pose_received_.z = msg->point.z;
        altitude_history_.push_back(pose_received_.z);
        // 自動旋回に入る直前の高度を記録、それを基準にして目標高度を決定
        if (altitude_history_.size() > 10)
        {
            altitude_history_.pop_front();
        }
    }

    void auto_turning_control()
    {
        // 制御値を計算
        double throttle = auto_turning_target.throttle_target;
        double elevator = neutral_position_.elevator + auto_turning_gain.elevator_gain * (auto_turning_target.altitude_target - pose_received_.z);
        double aileron_r = neutral_position_.aileron_r + auto_turning_gain.aileron_gain * (auto_turning_target.roll_target - pose_received_.roll);
        double aileron_l = neutral_position_.aileron_l + auto_turning_gain.aileron_gain * (auto_turning_target.roll_target - pose_received_.roll);
        double rudder;

        // ラダーの制御値を遅延させる
        if (auto_turning_delay.delay_rudder_counter > auto_turning_delay.delay_rudder)
        {
            rudder = auto_turning_target.rudder_target;
        }
        else
        {
            auto_turning_delay.delay_rudder_counter++;
            rudder = neutral_position_.rudder;
        }

        // 制御値を送信
        nokolat2024_msg::msg::Command command;
        command.throttle = cutoff_min_max(throttle, config.throttle_min, config.throttle_max);
        command.elevator = cutoff_min_max(elevator, config.elevator_min, config.elevator_max);
        command.aileron_r = cutoff_min_max(aileron_r, config.aileron_min_r, config.aileron_max_r);
        command.aileron_l = cutoff_min_max(aileron_l, config.aileron_min_l, config.aileron_max_l);
        command.rudder = cutoff_min_max(rudder, config.rudder_min, config.rudder_max);
        command.dropping_device = config.drop_min; // ドロップ装置は常に閉じておく
        command_publisher_->publish(command);
    }

    void auto_eight_control()
    {
        // 制御値を計算
    }

    double cutoff_min_max(double value, double min, double max)
    {
        if (value < min)
        {
            return min;
        }
        else if (value > max)
        {
            return max;
        }
        else
        {
            return value;
        }
    }

    void get_target_altitude()
    {
        auto_turning_target.altitude_target = std::accumulate(altitude_history_.begin(), altitude_history_.end(), 0.0) / altitude_history_.size();
    }

    // 制御情報を格納
    nokolat2024::main_control::ControlInfo_config config;
    nokolat2024::main_control::ControlInfo_gain auto_turning_gain;
    nokolat2024::main_control::ControlInfo_target auto_turning_target;
    nokolat2024::main_control::DelayWindow auto_turning_delay;

    nokolat2024::main_control::Pose pose_received_;
    nokolat2024::main_control::Command neutral_position_;

    // パラメータを取得
    YAML::Node control_info_config_;

    std::string control_mode_;

    bool mode_change_flag = false;
    std::string last_mode;

    std::deque<double> altitude_history_;
    std::deque<double> throttle_history_;

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr neutral_position_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr command_explicit_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitude_subscriber_;

    rclcpp::Publisher<nokolat2024_msg::msg::Command>::SharedPtr command_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainControlNode>());
    rclcpp::shutdown();
    return 0;
}