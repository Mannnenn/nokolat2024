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
        this->declare_parameter<std::string>("input_rotation_counter_topic_name", "/counter");

        this->declare_parameter<std::string>("output_command_topic_name", "/command_send");
        this->declare_parameter<std::string>("output_counter_reset_topic_name", "/counter_rest");

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
        std::string input_rotation_counter_topic_name;
        this->get_parameter("input_rotation_counter_topic_name", input_rotation_counter_topic_name);

        std::string output_command_topic_name;
        this->get_parameter("output_command_topic_name", output_command_topic_name);
        std::string output_counter_reset_topic_name;
        this->get_parameter("output_counter_reset_topic_name", output_counter_reset_topic_name);

        rclcpp::QoS qos(100); // 10 is the history depth
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // SubscriberとPublisherの設定
        neutral_position_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Command>(input_neutral_position_topic_name, qos, std::bind(&MainControlNode::neutral_position_callback, this, std::placeholders::_1));
        command_explicit_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Command>(input_command_explicit_topic_name, qos, std::bind(&MainControlNode::command_explicit_callback, this, std::placeholders::_1));
        mode_subscriber_ = this->create_subscription<std_msgs::msg::String>(input_mode_topic_name, qos, std::bind(&MainControlNode::mode_callback, this, std::placeholders::_1));
        rpy_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Rpy>(input_angular_topic_name, qos, std::bind(&MainControlNode::rpy_callback, this, std::placeholders::_1));
        altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(input_altitude_stamped_topic_name, qos, std::bind(&MainControlNode::altitude_callback, this, std::placeholders::_1));
        rotation_counter_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(input_rotation_counter_topic_name, qos, std::bind(&MainControlNode::rotation_count_callback, this, std::placeholders::_1));

        command_publisher_ = this->create_publisher<nokolat2024_msg::msg::Command>(output_command_topic_name, qos);
        counter_reset_publisher_ = this->create_publisher<std_msgs::msg::Float32>(output_counter_reset_topic_name, qos);

        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        throttle_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/throttle_command", qos_settings);
        altitude_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/altitude_target", qos_settings);
        altitude_error_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/altitude_error", qos_settings);
        pitch_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/pitch_target", qos_settings);
        pitch_error_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/pitch_error", qos_settings);
        elevator_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/elevator_command", qos_settings);
        roll_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/roll_target", qos_settings);
        roll_error_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/roll_error", qos_settings);
        aileron_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/aileron_command", qos_settings);
        rudder_command_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/rudder_command", qos_settings);

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
        auto_straight_gain.elevator_gain = control_info_config_["auto_straight"]["gain"]["elevator"]["p"].as<double>();
        auto_straight_gain.aileron_gain = control_info_config_["auto_straight"]["gain"]["aileron"]["p"].as<double>();
        auto_straight_gain.pitch_gain = control_info_config_["auto_straight"]["gain"]["pitch"]["p"].as<double>();

        auto_turning_gain.l.elevator_gain = control_info_config_["auto_turning"]["left"]["gain"]["elevator"]["p"].as<double>();
        auto_turning_gain.l.aileron_gain = control_info_config_["auto_turning"]["left"]["gain"]["aileron"]["p"].as<double>();
        auto_turning_gain.l.elevator_gain = control_info_config_["auto_turning"]["left"]["gain"]["elevator"]["p"].as<double>();

        auto_turning_gain.r.elevator_gain = control_info_config_["auto_turning"]["right"]["gain"]["elevator"]["p"].as<double>();
        auto_turning_gain.r.aileron_gain = control_info_config_["auto_turning"]["right"]["gain"]["aileron"]["p"].as<double>();
        auto_turning_gain.r.elevator_gain = control_info_config_["auto_turning"]["right"]["gain"]["elevator"]["p"].as<double>();

        eight_turning_gain.l.elevator_gain = control_info_config_["eight_turning"]["left"]["gain"]["elevator"]["p"].as<double>();
        eight_turning_gain.l.aileron_gain = control_info_config_["eight_turning"]["left"]["gain"]["aileron"]["p"].as<double>();
        eight_turning_gain.l.pitch_gain = control_info_config_["eight_turning"]["left"]["gain"]["pitch"]["p"].as<double>();

        eight_turning_gain.r.elevator_gain = control_info_config_["eight_turning"]["right"]["gain"]["elevator"]["p"].as<double>();
        eight_turning_gain.r.aileron_gain = control_info_config_["eight_turning"]["right"]["gain"]["aileron"]["p"].as<double>();
        eight_turning_gain.r.pitch_gain = control_info_config_["eight_turning"]["right"]["gain"]["pitch"]["p"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get gain parameter");

        // 制御目標値を取得
        auto_straight_target.roll_target = control_info_config_["auto_straight"]["target"]["roll"].as<double>();

        auto_turning_target.l.roll_target = control_info_config_["auto_turning"]["left"]["target"]["roll"].as<double>();
        auto_turning_target.l.rudder_target = control_info_config_["auto_turning"]["left"]["target"]["rudder"].as<double>();

        auto_turning_target.r.roll_target = control_info_config_["auto_turning"]["right"]["target"]["roll"].as<double>();
        auto_turning_target.r.rudder_target = control_info_config_["auto_turning"]["right"]["target"]["rudder"].as<double>();

        eight_turning_target.l.roll_target = control_info_config_["eight_turning"]["left"]["target"]["roll"].as<double>();
        eight_turning_target.l.rudder_target = control_info_config_["eight_turning"]["left"]["target"]["rudder"].as<double>();
        eight_turning_target.r.roll_target = control_info_config_["eight_turning"]["right"]["target"]["roll"].as<double>();
        eight_turning_target.r.rudder_target = control_info_config_["eight_turning"]["right"]["target"]["rudder"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get target parameter");
        auto_turning_delay.l.delay_rudder = control_info_config_["auto_turning"]["left"]["delay"]["rudder"].as<double>();
        auto_turning_delay.r.delay_rudder = control_info_config_["auto_turning"]["right"]["delay"]["rudder"].as<double>();

        eight_turning_delay.l.delay_rudder = control_info_config_["eight_turning"]["left"]["delay"]["rudder"].as<double>();
        eight_turning_delay.l.delay_rudder = control_info_config_["eight_turning"]["right"]["delay"]["rudder"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get delay window parameter");
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
    }

    void rpy_callback(const nokolat2024_msg::msg::Rpy::SharedPtr msg)

    {
        pose_received_.roll = msg->roll;
        pose_received_.pitch = msg->pitch;
        pose_received_.yaw = msg->yaw;
        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::MANUAL))
        {
            auto_straight_first_callback_flag = true;
            auto_turning_first_callback_flag = true;
            auto_turning_r_first_callback_flag = true;
            eight_turning_first_callback_flag = true;
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_STRAIGHT))
        {
            if (auto_straight_first_callback_flag)
            {
                // auto_turning_target.throttle_target = 692;
                RCLCPP_INFO(this->get_logger(), "MODE CHANGE");
                get_target_altitude();
                get_target_throttle();

                pub_rotation_rest_msg(1); // 1周したらカウントアップ

                auto_straight_first_callback_flag = false;
                RCLCPP_INFO(this->get_logger(), "target_altitude[%f]", auto_turning_target.altitude_target);
                RCLCPP_INFO(this->get_logger(), "target_throttle[%f]", auto_turning_target.throttle_target);
            }

            auto_straight_control();
            //   RCLCPP_INFO(this->get_logger(), "AUTO_TURNING");
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_TURNING))
        {
            if (auto_turning_first_callback_flag)
            {
                RCLCPP_INFO(this->get_logger(), "MODE CHANGE");
                get_target_altitude();
                get_target_throttle();

                pub_rotation_rest_msg(1); // 1周したらカウントアップに設定

                rudder_delay_time = this->now(); // ラダーの遅延起動測定のため

                auto_turning_first_callback_flag = false;
                RCLCPP_INFO(this->get_logger(), "target_altitude[%f]", auto_turning_target.altitude_target);
                RCLCPP_INFO(this->get_logger(), "target_throttle[%f]", auto_turning_target.throttle_target);
            }

            auto_turning_control();
            //   RCLCPP_INFO(this->get_logger(), "AUTO_TURNING");
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_TURNING_R))
        {
            if (auto_turning_r_first_callback_flag)
            {
                RCLCPP_INFO(this->get_logger(), "MODE CHANGE");
                get_target_altitude();
                get_target_throttle();

                pub_rotation_rest_msg(1); // 1周したらカウントアップに設定

                rudder_delay_time = this->now();

                auto_turning_r_first_callback_flag = false;
                RCLCPP_INFO(this->get_logger(), "target_altitude[%f]", auto_turning_target.altitude_target);
                RCLCPP_INFO(this->get_logger(), "target_throttle[%f]", auto_turning_target.throttle_target);
            }

            auto_turning_r_control();
            //   RCLCPP_INFO(this->get_logger(), "AUTO_TURNING");
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_EIGHT))
        {
            if (eight_turning_first_callback_flag)
            {
                get_target_altitude();
                get_target_throttle();

                pub_rotation_rest_msg(0.75); // 0.75周したらカウントアップに設定

                rudder_delay_time = this->now();

                turn_mode = nokolat2024::main_control::EIGHT_TURN_MODE::LEFT_TURN; // 最初は左旋回から

                eight_turning_first_callback_flag = false;
            }
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

    void rotation_count_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        rotation_count_ = msg->data;
    }

    void pub_ui_values()
    {
        std_msgs::msg::Float32 throttle_command_msg;
        throttle_command_msg.data = throttle;
        throttle_command_publisher_->publish(throttle_command_msg);

        std_msgs::msg::Float32 altitude_target_msg;
        altitude_target_msg.data = auto_turning_target.altitude_target;
        altitude_target_publisher_->publish(altitude_target_msg);

        std_msgs::msg::Float32 altitude_error_msg;
        altitude_error_msg.data = altitude_error;
        altitude_error_publisher_->publish(altitude_error_msg);

        std_msgs::msg::Float32 target_pitch_msg;
        target_pitch_msg.data = target_pitch;
        pitch_target_publisher_->publish(target_pitch_msg);

        std_msgs::msg::Float32 pitch_error_msg;
        pitch_error_msg.data = pitch_error;
        pitch_error_publisher_->publish(pitch_error_msg);

        std_msgs::msg::Float32 elevator_command_msg;
        elevator_command_msg.data = elevator;
        elevator_command_publisher_->publish(elevator_command_msg);

        std_msgs::msg::Float32 roll_target_msg;
        roll_target_msg.data = auto_turning_target.roll_target;
        roll_target_publisher_->publish(roll_target_msg);

        std_msgs::msg::Float32 roll_error_msg;
        roll_error_msg.data = roll_error;
        roll_error_publisher_->publish(roll_error_msg);

        std_msgs::msg::Float32 aileron_command_msg;
        aileron_command_msg.data = aileron_l;
        aileron_command_publisher_->publish(aileron_command_msg);

        std_msgs::msg::Float32 rudder_command_msg;
        rudder_command_msg.data = rudder;
        rudder_command_publisher_->publish(rudder_command_msg);
    }

    void pub_rotation_rest_msg(double standard_lap)
    {
        std_msgs::msg::Float32 rest_msg;
        rest_msg.data = standard_lap;
        counter_reset_publisher_->publish(rest_msg);
    }

    void auto_straight_control()
    {
        // 制御値を計算
        // スロットルは直前の値を使用
        throttle = throttle_avg;

        // 高度誤差とピッチ誤差を計算
        altitude_error = pose_received_.z - altitude_avg_;
        target_pitch = cutoff_min_max(auto_straight_gain.pitch_gain * altitude_error, -M_PI / 6, M_PI / 6);
        pitch_error = pose_received_.pitch - target_pitch;
        elevator = neutral_position_.elevator - auto_straight_gain.elevator_gain * pitch_error;

        // ロール誤差を計算
        roll_error = pose_received_.roll - auto_straight_target.roll_target;
        aileron_l = neutral_position_.aileron_l + auto_straight_gain.aileron_gain * roll_error;
        aileron_r = neutral_position_.aileron_r + auto_straight_gain.aileron_gain * roll_error;

        // ラッダーはそのまま
        rudder = neutral_position_.rudder;

        // 制御値を送信
        nokolat2024_msg::msg::Command command;
        command.throttle = cutoff_min_max(throttle, config.throttle_min, config.throttle_max);
        command.elevator = cutoff_min_max(elevator, config.elevator_min, config.elevator_max);
        command.aileron_r = cutoff_min_max(aileron_r, config.aileron_min_r, config.aileron_max_r);
        command.aileron_l = cutoff_min_max(aileron_l, config.aileron_min_l, config.aileron_max_l);
        command.rudder = cutoff_min_max(rudder, config.rudder_min, config.rudder_max);
        command.dropping_device = config.drop_min; // ドロップ装置は常に閉じておく
        command_publisher_->publish(command);

        pub_ui_values();
    }

    void auto_turning_control()
    {
        // 制御値を計算
        // スロットルは直前の値を使用
        throttle = throttle_avg;

        // 高度誤差とピッチ誤差を計算
        altitude_error = pose_received_.z - altitude_avg_;
        target_pitch = cutoff_min_max(auto_turning_gain.l.pitch_gain * altitude_error, -M_PI / 6, M_PI / 6);
        pitch_error = pose_received_.pitch - target_pitch;
        elevator = neutral_position_.elevator - auto_turning_gain.l.elevator_gain * pitch_error;

        // ロール誤差を計算
        roll_error = pose_received_.roll - auto_turning_target.l.roll_target;
        aileron_l = neutral_position_.aileron_l + auto_turning_gain.l.aileron_gain * roll_error;
        aileron_r = neutral_position_.aileron_r + auto_turning_gain.l.aileron_gain * roll_error;

        // ラダーの制御値を遅延させる
        rclcpp::Duration dt = this->now() - rudder_delay_time;
        if (dt.seconds() < auto_turning_delay.l.delay_rudder) // 旋回司令が送られてから暫くまつ
        {
            rudder = auto_turning_target.l.rudder_target;
        }
        else
        {
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

        pub_ui_values();
    }

    void auto_turning_r_control()
    {
        // 制御値を計算
        // スロットルは直前の値を使用
        throttle = throttle_avg;

        // 高度誤差とピッチ誤差を計算
        altitude_error = pose_received_.z - altitude_avg_;
        target_pitch = cutoff_min_max(auto_turning_gain.r.pitch_gain * altitude_error, -M_PI / 6, M_PI / 6);
        pitch_error = pose_received_.pitch - target_pitch;
        elevator = neutral_position_.elevator - auto_turning_gain.r.elevator_gain * pitch_error;

        // ロール誤差を計算
        roll_error = pose_received_.roll - auto_turning_target.r.roll_target;
        aileron_l = neutral_position_.aileron_l + auto_turning_gain.r.aileron_gain * roll_error;
        aileron_r = neutral_position_.aileron_r + auto_turning_gain.r.aileron_gain * roll_error;

        // ラダーの制御値を遅延させる
        rclcpp::Duration dt = this->now() - rudder_delay_time;
        if (dt.seconds() < auto_turning_delay.r.delay_rudder) // 旋回司令が送られてから暫くまつ
        {
            rudder = auto_turning_target.r.rudder_target;
        }
        else
        {
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

        pub_ui_values();
    }

    void auto_eight_control()
    {
        if (turning_count == 1) // 左旋回で270deg回ると1になる
        {
            if (turn_mode == nokolat2024::main_control::EIGHT_TURN_MODE::LEFT_TURN)
            {
                turn_mode = nokolat2024::main_control::EIGHT_TURN_MODE::NEUTRAL;
                get_horizontal_roll_time = this->now();
                eight_turning_target.l.roll_target = pose_received_.roll; // しばらくこの姿勢を維持
                eight_turning_target.yaw_target = pose_received_.yaw;     // 直進する
            }
        }
        if (turning_count == -1) // 右旋回で270deg回ると-1になる
        {
            if (turn_mode == nokolat2024::main_control::EIGHT_TURN_MODE::RIGHT_TURN)
            {
                turn_mode = nokolat2024::main_control::EIGHT_TURN_MODE::NEUTRAL;
                get_horizontal_roll_time = this->now();
                eight_turning_target.l.roll_target = pose_received_.roll; // しばらくこの姿勢を維持
                eight_turning_target.yaw_target = pose_received_.yaw;     // 直進する
            }
        }

        if (turn_mode == nokolat2024::main_control::EIGHT_TURN_MODE::LEFT_TURN)
        {

            // 左旋回
            // 制御値を計算
            throttle = throttle_avg;
            elevator = neutral_position_.elevator + eight_turning_gain.l.elevator_gain * (altitude_avg_ - pose_received_.z);
            aileron_r = neutral_position_.aileron_r + eight_turning_gain.l.aileron_gain * (eight_turning_target.l.roll_target - pose_received_.roll); // 左旋回時用のパラメーター
            aileron_l = neutral_position_.aileron_l + eight_turning_gain.l.aileron_gain * (eight_turning_target.l.roll_target - pose_received_.roll);

            // ラダーの制御値を遅延させる
            rclcpp::Duration dt = this->now() - rudder_delay_time;
            if (dt.seconds() < eight_turning_delay.l.delay_rudder) // 旋回司令が送られてから暫くまつ
            {
                rudder = eight_turning_target.l.rudder_target;
            }
            else
            {
                rudder = neutral_position_.rudder;
            }
        }

        // 水平に戻す
        if (turn_mode == nokolat2024::main_control::EIGHT_TURN_MODE::NEUTRAL)
        {
            // ラダー解除→ロール水平化
            // 制御値を計算
            throttle = throttle_avg;
            elevator = neutral_position_.elevator + eight_turning_gain.l.elevator_gain * (altitude_avg_ - pose_received_.z);

            // ラダーの制御値を遅延させる
            rclcpp::Duration dt = this->now() - rudder_delay_time;
            if (dt.seconds() < eight_turning_delay.l.delay_rudder) // しばらく待つ。ラダーはニュートラルの位置に、ロールは維持
            {
                rudder = neutral_position_.rudder;
                aileron_r = neutral_position_.aileron_r + eight_turning_gain.l.aileron_gain * (eight_turning_target.roll_target - pose_received_.roll);
                aileron_l = neutral_position_.aileron_l + eight_turning_gain.l.aileron_gain * (eight_turning_target.roll_target - pose_received_.roll);
            }
            else // しばらく待ったら水平に
            {
                eight_turning_target.roll_target = 0;

                rudder = neutral_position_.rudder;
                aileron_r = neutral_position_.aileron_r + eight_turning_gain.l.aileron_gain * (eight_turning_target.roll_target - pose_received_.roll);
                aileron_l = neutral_position_.aileron_l + eight_turning_gain.l.aileron_gain * (eight_turning_target.roll_target - pose_received_.roll);
            }
        }

        if (turn_mode == nokolat2024::main_control::EIGHT_TURN_MODE::RIGHT_TURN)
        {

            // 右旋回
            // 制御値を計算
            throttle = throttle_avg;
            elevator = neutral_position_.elevator + eight_turning_gain.elevator_gain * (altitude_avg_ - pose_received_.z);
            aileron_r = neutral_position_.aileron_r + eight_turning_gain.aileron_gain * (eight_turning_target.r.roll_target - pose_received_.roll); // 左旋回時用のパラメーター
            aileron_l = neutral_position_.aileron_l + eight_turning_gain.aileron_gain * (eight_turning_target.r.roll_target - pose_received_.roll);

            // ラダーの制御値を遅延させる
            // ラダーの制御値を遅延させる
            rclcpp::Duration dt = this->now() - rudder_delay_time;
            if (dt.seconds() < eight_turning_delay.r.delay_rudder) // 旋回司令が送られてから暫くまつ
            {
                rudder = eight_turning_target.r.rudder_target;
            }
            else
            {
                rudder = neutral_position_.rudder;
            }
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
        altitude_avg_ = std::accumulate(altitude_history_.begin(), altitude_history_.end(), 0.0) / altitude_history_.size();
    }

    void get_target_throttle()
    {
        throttle_avg = std::accumulate(throttle_history_.begin(), throttle_history_.end(), 0.0) / throttle_history_.size();
        RCLCPP_INFO(this->get_logger(), "target_throttle[%f]", auto_turning_target.throttle_target);
    }

    // 制御情報を格納
    nokolat2024::main_control::ControlInfo_config config;

    nokolat2024::main_control::ControlInfo_gain auto_straight_gain;
    nokolat2024::main_control::ControlInfo_target auto_straight_target;

    nokolat2024::main_control::ControlInfo_gain_lr auto_turning_gain;
    nokolat2024::main_control::ControlInfo_target_lr auto_turning_target;
    nokolat2024::main_control::DelayWindow_lr auto_turning_delay;

    nokolat2024::main_control::ControlInfo_gain_lr eight_turning_gain;
    nokolat2024::main_control::ControlInfo_target_lr eight_turning_target;
    nokolat2024::main_control::DelayWindow_lr eight_turning_delay;

    nokolat2024::main_control::Pose pose_received_;
    nokolat2024::main_control::Command neutral_position_;

    // パラメータを取得
    YAML::Node control_info_config_;

    std::string control_mode_;

    bool auto_straight_first_callback_flag = false;
    bool auto_turning_first_callback_flag = false;
    bool auto_turning_r_first_callback_flag = false;
    bool eight_turning_first_callback_flag = false;

    std::deque<double> altitude_history_;
    std::deque<double> throttle_history_;
    std::deque<double> yaw_history_;

    constexpr static double yaw_delta = 0.05 * M_PI;
    bool count_start_flag = true;
    int yaw_offset_flag = 0;
    double count_start_yaw = 0;
    double turning_count_range = 1.75 * M_PI;
    double turning_count = 0;
    uint last_count;

    double throttle_avg;
    double altitude_avg_;

    double throttle;
    double altitude_error;
    double target_pitch;
    double pitch_error;
    double roll_error;
    double elevator;
    double aileron_l;
    double aileron_r;
    double rudder;

    double rotation_count_;
    nokolat2024::main_control::EIGHT_TURN_MODE turn_mode;
    rclcpp::Time get_horizontal_roll_time;

    rclcpp::Time rudder_delay_time;

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr neutral_position_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr command_explicit_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitude_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rotation_counter_subscriber_;

    rclcpp::Publisher<nokolat2024_msg::msg::Command>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr counter_reset_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aileron_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_command_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainControlNode>());
    rclcpp::shutdown();
    return 0;
}