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

        this->declare_parameter<std::string>("input_rotation_counter_topic_name", "/rotation_counter");

        this->declare_parameter<std::string>("input_cmd_vel_topic_name", "/cmd_vel");
        this->declare_parameter<std::string>("input_drop_timing_topic_name", "/target_pose");
        this->declare_parameter<std::string>("input_throttle_off_timing_topic_name", "/throttle_off");

        this->declare_parameter<std::string>("output_command_topic_name", "/command_send");
        this->declare_parameter<std::string>("output_counter_reset_topic_name", "/rotation_counter_reset");

        // パラメータの取得
        std::string input_neutral_position_topic_name;
        this->get_parameter("input_neutral_position_topic_name", input_neutral_position_topic_name);

        std::string input_command_explicit_topic_name;
        this->get_parameter("input_command_explicit_topic_name", input_command_explicit_topic_name);
        std::string input_mode_topic_name;
        this->get_parameter("input_mode_topic_name", input_mode_topic_name);
        std::string input_angular_topic_name;
        this->get_parameter("input_angular_topic_name", input_angular_topic_name);

        std::string input_rotation_counter_topic_name;
        this->get_parameter("input_rotation_counter_topic_name", input_rotation_counter_topic_name);

        std::string input_cmd_vel_topic_name;
        this->get_parameter("input_cmd_vel_topic_name", input_cmd_vel_topic_name);
        std::string input_drop_timing_topic_name;
        this->get_parameter("input_drop_timing_topic_name", input_drop_timing_topic_name);
        std::string input_throttle_off_timing_topic_name;
        this->get_parameter("input_throttle_off_timing_topic_name", input_throttle_off_timing_topic_name);

        std::string output_command_topic_name;
        this->get_parameter("output_command_topic_name", output_command_topic_name);
        std::string output_counter_reset_topic_name;
        this->get_parameter("output_counter_reset_topic_name", output_counter_reset_topic_name);

        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        // SubscriberとPublisherの設定
        neutral_position_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Command>(input_neutral_position_topic_name, qos, std::bind(&MainControlNode::neutral_position_callback, this, std::placeholders::_1));

        command_explicit_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Command>(input_command_explicit_topic_name, qos, std::bind(&MainControlNode::command_explicit_callback, this, std::placeholders::_1));
        mode_subscriber_ = this->create_subscription<std_msgs::msg::String>(input_mode_topic_name, qos, std::bind(&MainControlNode::mode_callback, this, std::placeholders::_1));
        rpy_subscriber_ = this->create_subscription<nokolat2024_msg::msg::Rpy>(input_angular_topic_name, qos, std::bind(&MainControlNode::rpy_callback, this, std::placeholders::_1));

        rotation_counter_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(input_rotation_counter_topic_name, qos, std::bind(&MainControlNode::rotation_counter_callback, this, std::placeholders::_1));

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(input_cmd_vel_topic_name, qos, std::bind(&MainControlNode::cmd_vel_callback, this, std::placeholders::_1));
        drop_timing_subscriber_ = this->create_subscription<std_msgs::msg::String>(input_drop_timing_topic_name, qos, std::bind(&MainControlNode::drop_timing_callback, this, std::placeholders::_1));
        throttle_off_timing_subscriber_ = this->create_subscription<std_msgs::msg::String>(input_throttle_off_timing_topic_name, qos, std::bind(&MainControlNode::throttle_off_timing_callback, this, std::placeholders::_1));

        command_publisher_ = this->create_publisher<nokolat2024_msg::msg::Command>(output_command_topic_name, qos);
        rotation_counter_reset_publisher_ = this->create_publisher<std_msgs::msg::Float32>(output_counter_reset_topic_name, qos);

        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        altitude_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/altitude_target", qos_settings);
        altitude_error_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/altitude_error", qos_settings);
        pitch_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/pitch_target", qos_settings);
        pitch_error_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/pitch_error", qos_settings);
        roll_target_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/roll_target", qos_settings);
        roll_error_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/ui/roll_error", qos_settings);

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

        config.throttle_max = control_info_config_["chassis"]["throttle"]["max"].as<double>();
        config.throttle_min = control_info_config_["chassis"]["throttle"]["min"].as<double>();
        config.elevator_max = control_info_config_["chassis"]["elevator"]["max"].as<double>();
        config.elevator_min = control_info_config_["chassis"]["elevator"]["min"].as<double>();
        config.aileron_max_r = control_info_config_["chassis"]["aileron"]["right"]["max"].as<double>();
        config.aileron_min_r = control_info_config_["chassis"]["aileron"]["right"]["min"].as<double>();
        config.aileron_max_l = control_info_config_["chassis"]["aileron"]["left"]["max"].as<double>();
        config.aileron_min_l = control_info_config_["chassis"]["aileron"]["left"]["min"].as<double>();
        config.rudder_max = control_info_config_["chassis"]["rudder"]["max"].as<double>();
        config.rudder_min = control_info_config_["chassis"]["rudder"]["min"].as<double>();
        config.drop_max = control_info_config_["chassis"]["drop"]["max"].as<double>();
        config.drop_center = control_info_config_["chassis"]["drop"]["center"].as<double>();
        config.drop_min = control_info_config_["chassis"]["drop"]["min"].as<double>();
        RCLCPP_INFO(this->get_logger(), "get chassis parameter");

        // 収束判断の閾値を取得
        roll_convergence_threshold = control_info_config_["convergence"]["roll"].as<double>();
        altitude_convergence_threshold = control_info_config_["convergence"]["altitude"].as<double>();

        // ゲインなどのパラメータを取得
        auto_turning_gain.elevator_gain = control_info_config_["auto_turning"]["gain"]["elevator"]["p"].as<double>();
        auto_turning_gain.aileron_gain = control_info_config_["auto_turning"]["gain"]["aileron"]["p"].as<double>();
        auto_turning_gain.pitch_gain = control_info_config_["auto_turning"]["gain"]["pitch"]["p"].as<double>();
        auto_turning_gain.nose_up_pitch_gain = control_info_config_["auto_turning"]["gain"]["nose_up_pitch"].as<double>();

        rise_turning_gain.elevator_gain = control_info_config_["rise_turning"]["gain"]["elevator"]["p"].as<double>();
        rise_turning_gain.aileron_gain = control_info_config_["rise_turning"]["gain"]["aileron"]["p"].as<double>();
        rise_turning_gain.pitch_gain = control_info_config_["rise_turning"]["gain"]["pitch"]["p"].as<double>();
        rise_turning_gain.nose_up_pitch_gain = control_info_config_["rise_turning"]["gain"]["nose_up_pitch"].as<double>();

        eight_turning_gain.elevator_gain = control_info_config_["eight_turning"]["gain"]["elevator"]["p"].as<double>();
        eight_turning_gain.aileron_gain = control_info_config_["eight_turning"]["gain"]["aileron"]["p"].as<double>();
        eight_turning_gain.pitch_gain = control_info_config_["eight_turning"]["gain"]["pitch"]["p"].as<double>();
        eight_turning_gain.nose_up_pitch_gain = control_info_config_["eight_turning"]["gain"]["nose_up_pitch"].as<double>();

        auto_landing_gain.elevator_gain = control_info_config_["auto_landing"]["gain"]["elevator"]["p"].as<double>();
        auto_landing_gain.aileron_gain = control_info_config_["auto_landing"]["gain"]["aileron"]["p"].as<double>();
        auto_landing_gain.rudder_gain = control_info_config_["auto_landing"]["gain"]["rudder"]["p"].as<double>();
        auto_landing_gain.pitch_gain = control_info_config_["auto_landing"]["gain"]["pitch"]["p"].as<double>();
        auto_landing_gain.nose_up_pitch_gain = control_info_config_["auto_landing"]["gain"]["nose_up_pitch"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get gain parameter");

        // 制御目標値を取得
        auto_turning_target.roll_target = control_info_config_["auto_turning"]["target"]["roll"].as<double>();
        auto_turning_target.rudder_target = control_info_config_["auto_turning"]["target"]["rudder"].as<double>();
        auto_turning_target.pitch_target = control_info_config_["auto_turning"]["target"]["pitch"].as<double>();

        rise_turning_target.roll_target = control_info_config_["rise_turning"]["target"]["roll"].as<double>();
        rise_turning_target.rudder_target = control_info_config_["rise_turning"]["target"]["rudder"].as<double>();
        rise_turning_target.pitch_target = control_info_config_["rise_turning"]["target"]["pitch"].as<double>();
        rise_turning_target.higher_altitude_target = control_info_config_["rise_turning"]["target"]["upper_altitude"].as<double>();
        rise_turning_target.rise_throttle_target = control_info_config_["rise_turning"]["target"]["rise"]["throttle"].as<double>();
        rise_turning_target.rise_roll_target = control_info_config_["rise_turning"]["target"]["rise"]["roll"].as<double>();
        rise_turning_target.rise_rudder_target = control_info_config_["rise_turning"]["target"]["rise"]["rudder"].as<double>();
        rise_turning_target.rise_pitch_target = control_info_config_["rise_turning"]["target"]["rise"]["pitch"].as<double>();

        eight_turning_target.roll_target_l = control_info_config_["eight_turning"]["target"]["roll"]["l"].as<double>();
        eight_turning_target.roll_target_r = control_info_config_["eight_turning"]["target"]["roll"]["r"].as<double>();
        eight_turning_target.rudder_target_l = control_info_config_["eight_turning"]["target"]["rudder"]["l"].as<double>();
        eight_turning_target.rudder_target_r = control_info_config_["eight_turning"]["target"]["rudder"]["r"].as<double>();
        eight_turning_target.pitch_target_l = control_info_config_["eight_turning"]["target"]["pitch"]["l"].as<double>();
        eight_turning_target.pitch_target_r = control_info_config_["eight_turning"]["target"]["pitch"]["r"].as<double>();
        eight_turning_target.pitch_target_recover = control_info_config_["eight_turning"]["target"]["pitch"]["recover"].as<double>();

        auto_landing_target.throttle_target = control_info_config_["auto_landing"]["target"]["throttle"].as<double>();
        auto_landing_target.takeoff_throttle_target = control_info_config_["auto_landing"]["target"]["takeoff_throttle"].as<double>();
        auto_landing_target.roll_target = control_info_config_["auto_landing"]["target"]["roll"].as<double>();
        auto_landing_target.pitch_target = control_info_config_["auto_landing"]["target"]["pitch"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get target parameter");

        // 制御遅れを取得
        auto_turning_delay.delay_rudder = control_info_config_["auto_turning"]["delay"]["rudder"].as<double>();

        rise_turning_delay.delay_rudder = control_info_config_["rise_turning"]["delay"]["rudder"].as<double>();
        rise_turning_delay.delay_rise = control_info_config_["rise_turning"]["delay"]["throttle"].as<double>();
        rise_turning_delay.delay_decel = control_info_config_["rise_turning"]["delay"]["decel"].as<double>();

        eight_turning_delay.delay_rudder = control_info_config_["eight_turning"]["delay"]["rudder"].as<double>();

        auto_landing_delay.delay_accel = control_info_config_["auto_landing"]["delay"]["accel"].as<double>();
        auto_landing_delay.delay_decel = control_info_config_["auto_landing"]["delay"]["decel"].as<double>();
        auto_landing_delay.delay_land = control_info_config_["auto_landing"]["delay"]["land"].as<double>();

        RCLCPP_INFO(this->get_logger(), "get delay window parameter");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // base_linkが利用可能になるまで待機
        while (rclcpp::ok())
        {
            try
            {
                tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "base_link is now available.");
                break;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for base_link to become available: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(75), // 早すぎると通信エラーが発生する
            std::bind(&MainControlNode::timer_callback, this));

        ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
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

        yaw_history_.push_back(msg->yaw);
        if (yaw_history_.size() > 5)
        {
            yaw_history_.pop_front();
        }
    }

    void rotation_counter_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        turning_count = msg->data;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto_landing_target.induction_altitude_target = msg->linear.z;
        auto_landing_target.induction_pitch_diff = -msg->angular.y;
        auto_landing_target.induction_yaw_diff = msg->angular.z;
    }

    void drop_timing_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "drop")
        {
            drop_timing = true;
        }
        else
        {
            drop_timing = false;
        }
    }

    void throttle_off_timing_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "throttle_off")
        {
            throttle_off_timing = true;
        }
        else
        {
            throttle_off_timing = false;
        }
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        altitude_history_.push_back(transform_stamped.transform.translation.z);
        pose_received_.z = transform_stamped.transform.translation.z;

        // 自動旋回に入る直前の高度を記録、それを基準にして目標高度を決定
        if (altitude_history_.size() > 10)
        {
            altitude_history_.pop_front();
        }

        main_control();
    }

    void main_control()
    {
        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::MANUAL))
        {
            // それぞれのモードの初回呼び出し用。マニュアルに戻したら初期化
            mode_init = true;

            // 旋回カウンタもリセット
            reset_rotation_count(1);
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_TURNING))
        {
            if (mode_init)
            {
                RCLCPP_INFO(this->get_logger(), "MODE: AUTO_TURNING");

                auto_turning_target.altitude_target = get_target_altitude();
                auto_turning_target.throttle_target = get_target_throttle();
                mode_init = false;
                roll_align = false;
                start_mode_time_ = ros_clock->now(); // 旋回開始の時間を取得

                reset_rotation_count(1);
            }
            auto_turning_control();
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_RISE_TURNING))
        {
            if (mode_init)
            {
                RCLCPP_INFO(this->get_logger(), "MODE: AUTO_RISE_TURNING");
                rise_turning_target.lower_altitude_target = get_target_altitude();
                rise_turning_target.throttle_target = get_target_throttle();
                mode_init = false;
                roll_align = false;
                start_mode_time_ = ros_clock->now(); // 旋回開始の時間を取得

                // 低高度旋回から開始
                rise_turning_mode = nokolat2024::main_control::RISE_TURNING_MODE::LOWER_TURNING;

                reset_rotation_count(1);
            }
            auto_rise_turning_control();
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_EIGHT_TURNING))
        {
            if (mode_init)
            {
                RCLCPP_INFO(this->get_logger(), "MODE: AUTO_EIGHT_TURNING");
                eight_turning_target.altitude_target = get_target_altitude();
                eight_turning_target.throttle_target = get_target_throttle();
                mode_init = false;
                roll_align = false;
                start_mode_time_ = ros_clock->now(); // 旋回開始の時間を取得
                // 左旋回スタート
                eight_turning_mode = nokolat2024::main_control::EIGHT_TURNING_MODE::LEFT_TURNING;

                reset_rotation_count(0.75);
            }
            auto_eight_turning_control();
        }

        if (control_mode_ == nokolat2024::main_control::control_mode_map.at(nokolat2024::main_control::CONTROL_MODE::AUTO_LANDING))
        {
            if (mode_init)
            {
                RCLCPP_INFO(this->get_logger(), "MODE: AUTO_LANDING");

                // 自動離着陸用のタイミングもリセット
                drop_timing = false;
                throttle_off_timing = false;
                daccel_start = false;

                mode_init = false;
                start_mode_time_ = ros_clock->now(); // 開始の時間を取得
            }
            auto_landing_control();
        }

        pub_command_data();
        pub_ui_data();
    }

    double throttle_control(double throttle_target)
    {
        return throttle_target;
    }

    double elevator_control(double altitude_target, double pitch_gain, double pitch_target, double nose_up_pitch_gain, double elevator_gain)
    {
        double elevator;

        // 標高の誤差を計算
        altitude_error = pose_received_.z - altitude_target;
        // 標高の誤差にピッチを比例
        target_pitch = cutoff_min_max(pitch_gain * altitude_error + pitch_target, -M_PI / 6, M_PI / 6);
        // ピッチの誤差を計算
        pitch_error = pose_received_.pitch - target_pitch;

        // ピッチの誤差にエレベーターを比例、機首上げがよくなるように係数を変更
        if (target_pitch >= 0) // 機首下げ
        {
            elevator = neutral_position_.elevator + elevator_gain * pitch_error;
        }
        if (target_pitch < 0) // 機首上げ
        {
            elevator = neutral_position_.elevator + nose_up_pitch_gain * elevator_gain * pitch_error;
        }

        return elevator;
    }

    double aileron_control_l(double roll_target, double aileron_gain)
    {
        // ロールの誤差を計算
        roll_error = pose_received_.roll - roll_target;

        // ロールの誤差にエルロンを比例
        return neutral_position_.aileron_l + aileron_gain * roll_error;
    }

    double aileron_control_r(double roll_target, double aileron_gain)
    {
        // ロールの誤差を計算
        roll_error = pose_received_.roll - roll_target;

        // ロールの誤差にエルロンを比例
        return neutral_position_.aileron_r + aileron_gain * roll_error;
    }

    double rudder_control(double rudder_target, double rudder_delay, double prev_target = 0)
    {
        // ラダーの制御
        double rudder;

        // rollが目標値に収束したらラダーの制御を開始
        if (-roll_convergence_threshold < roll_error && roll_error < roll_convergence_threshold && !roll_align)
        {
            roll_align_time_ = ros_clock->now();
            roll_align = true;
        }
        if (roll_align)
        {
            rclcpp::Duration from_roll_align = ros_clock->now() - roll_align_time_;
            // 徐々にラダーの目標値を目標値に近づける
            // 上昇旋回時は定常旋回時より速度を上げるため、ラダーをより変化させる。変化前は(ニュートラル+定常)ラダー,変化後は(ニュートラル+定常+上昇)ラダー
            rudder = neutral_position_.rudder + prev_target + rudder_target * linearIncrease(from_roll_align.seconds(), rudder_delay);
        }
        else
        {
            rudder = neutral_position_.rudder;
        }

        return rudder;
    }

    double rudder_control_recover(double rudder_target, double rudder_delay, double prev_target = 0)
    {
        // ラダーの制御
        // ラダーを中立位置に戻す
        // 上昇後にもとの定常ラダーに戻す。変化前は(ニュートラル+定常+上昇)、ラダー変化後は(ニュートラル+定常)ラダー
        rclcpp::Duration from_roll_align = ros_clock->now() - recover_start_time_;
        return neutral_position_.rudder + prev_target + rudder_target * linearDecrease(from_roll_align.seconds(), rudder_delay);
    }

    double yaw_control(double yaw_target, double yaw_gain)
    {
        // ヨーの誤差を計算
        yaw_error = pose_received_.yaw - yaw_target;
        // ヨーの誤差にラダーを比例
        return neutral_position_.rudder + yaw_gain * yaw_error;
    }

    double drop_control(double drop_target)
    {
        return drop_target;
    }

    void reset_rotation_count(double lap_base)
    {
        for (int i = 0; i < 5; i++)
        {
            // lap_base周でカウントが変化するように
            pub_rotation_rest_data(lap_base);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void auto_turning_control()
    {
        // 制御値を計算
        throttle = throttle_control(auto_turning_target.throttle_target);
        elevator = elevator_control(auto_turning_target.altitude_target, eight_turning_gain.pitch_gain, auto_turning_target.pitch_target, auto_turning_gain.nose_up_pitch_gain, auto_turning_gain.elevator_gain);
        aileron_l = aileron_control_l(auto_turning_target.roll_target, auto_turning_gain.aileron_gain);
        aileron_r = aileron_control_r(auto_turning_target.roll_target, auto_turning_gain.aileron_gain);
        rudder = rudder_control(auto_turning_target.rudder_target, auto_turning_delay.delay_rudder);

        // 投下装置は閉じる
        drop = drop_control(config.drop_min);
    }

    void auto_rise_turning_control()
    {
        if (rise_turning_mode == nokolat2024::main_control::RISE_TURNING_MODE::LOWER_TURNING) // 低い高度で2周する,上昇が成功していない間は
        {
            throttle = throttle_control(rise_turning_target.throttle_target);
            elevator = elevator_control(rise_turning_target.lower_altitude_target, rise_turning_gain.pitch_gain, rise_turning_target.pitch_target, rise_turning_gain.nose_up_pitch_gain, rise_turning_gain.elevator_gain);
            aileron_l = aileron_control_l(rise_turning_target.roll_target, rise_turning_gain.aileron_gain);
            aileron_r = aileron_control_r(rise_turning_target.roll_target, rise_turning_gain.aileron_gain);
            rudder = rudder_control(rise_turning_target.rudder_target, rise_turning_delay.delay_rudder);

            if (abs(turning_count) >= 3)
            {
                rise_turning_mode = nokolat2024::main_control::RISE_TURNING_MODE::RISE_TURNING;
                start_mode_time_ = ros_clock->now();
                RCLCPP_INFO(this->get_logger(), "MODE::CHANGE TO RISE");
            }
        }

        if (rise_turning_mode == nokolat2024::main_control::RISE_TURNING_MODE::RISE_TURNING)
        {
            throttle = throttle_control(rise_turning_target.throttle_target + rise_turning_target.rise_throttle_target);

            // 徐々に目標高度を上げる
            rclcpp::Time current_time = ros_clock->now();
            rclcpp::Duration diff = current_time - start_mode_time_;
            rise_turning_target.altitude_target = rise_turning_target.lower_altitude_target + (rise_turning_target.higher_altitude_target - rise_turning_target.lower_altitude_target) * linearIncrease(diff.seconds(), rise_turning_delay.delay_rise);

            elevator = elevator_control(rise_turning_target.altitude_target, rise_turning_gain.pitch_gain, rise_turning_target.rise_pitch_target, rise_turning_gain.nose_up_pitch_gain, rise_turning_gain.elevator_gain);
            aileron_l = aileron_control_l(rise_turning_target.rise_roll_target, rise_turning_gain.aileron_gain);
            aileron_r = aileron_control_r(rise_turning_target.rise_roll_target, rise_turning_gain.aileron_gain);
            rudder = rudder_control(rise_turning_target.rise_rudder_target, rise_turning_delay.delay_rudder, rise_turning_target.rudder_target);

            if (get_target_altitude() > rise_turning_target.higher_altitude_target)
            {
                rise_turning_mode = nokolat2024::main_control::RISE_TURNING_MODE::HIGHER_TURNING;
                reset_rotation_count(1);
                start_mode_time_ = ros_clock->now();
                RCLCPP_INFO(this->get_logger(), "MODE::CHANGE TO TURN");
            }
        }

        if (rise_turning_mode == nokolat2024::main_control::RISE_TURNING_MODE::HIGHER_TURNING) // 低い高度で2周する,上昇が成功していない間は
        {
            rclcpp::Duration diff = ros_clock->now() - start_mode_time_;
            throttle = throttle_control(rise_turning_target.throttle_target + rise_turning_target.rise_throttle_target * linearDecrease(diff.seconds(), rise_turning_delay.delay_decel));
            elevator = elevator_control(rise_turning_target.higher_altitude_target, rise_turning_gain.pitch_gain, rise_turning_target.pitch_target + (rise_turning_target.rise_pitch_target - rise_turning_target.pitch_target) * linearIncrease(diff.seconds(), rise_turning_delay.delay_decel), rise_turning_gain.nose_up_pitch_gain, rise_turning_gain.elevator_gain);
            aileron_l = aileron_control_l(rise_turning_target.rise_roll_target, rise_turning_gain.aileron_gain);
            aileron_r = aileron_control_r(rise_turning_target.rise_roll_target, rise_turning_gain.aileron_gain);
            rudder = neutral_position_.rudder + rise_turning_target.rudder_target + rise_turning_target.rise_rudder_target;
        }

        drop = drop_control(config.drop_min);
    }

    void auto_eight_turning_control()
    {
        // 左旋回→左回復→右旋回→右回復→左旋回のサイクル
        // 左旋回中に回転数が-1になったら右旋回に移行,右旋回中に回転数が1になったら左旋回に移行

        if (eight_turning_mode == nokolat2024::main_control::EIGHT_TURNING_MODE::LEFT_TURNING)
        {

            // 制御値を計算
            throttle = throttle_control(eight_turning_target.throttle_target);
            elevator = elevator_control(eight_turning_target.altitude_target, eight_turning_gain.pitch_gain, eight_turning_target.pitch_target_l, eight_turning_gain.nose_up_pitch_gain, eight_turning_gain.elevator_gain);
            aileron_l = aileron_control_l(eight_turning_target.roll_target_l, eight_turning_gain.aileron_gain);
            aileron_r = aileron_control_r(eight_turning_target.roll_target_l, eight_turning_gain.aileron_gain);
            rudder = rudder_control(eight_turning_target.rudder_target_l, eight_turning_delay.delay_rudder);

            if (turning_count == 1) // 左旋回でこの値になる
            {
                eight_turning_mode = nokolat2024::main_control::EIGHT_TURNING_MODE::NEUTRAL_POSITION_L;
                reset_rotation_count(0.75);
                recover_start_time_ = ros_clock->now(); // 水平状態に戻り始める時間を取得
                RCLCPP_INFO(this->get_logger(), "LEFT END");
                roll_align = false;
            }
        }

        if (eight_turning_mode == nokolat2024::main_control::EIGHT_TURNING_MODE::RIGHT_TURNING)
        {
            // 制御値を計算
            throttle = throttle_control(eight_turning_target.throttle_target);
            elevator = elevator_control(eight_turning_target.altitude_target, eight_turning_gain.pitch_gain, eight_turning_target.pitch_target_r, eight_turning_gain.nose_up_pitch_gain, eight_turning_gain.elevator_gain);
            aileron_l = aileron_control_l(eight_turning_target.roll_target_r, eight_turning_gain.aileron_gain);
            aileron_r = aileron_control_r(eight_turning_target.roll_target_r, eight_turning_gain.aileron_gain);
            rudder = rudder_control(eight_turning_target.rudder_target_r, eight_turning_delay.delay_rudder);

            if (turning_count == -1) // 右旋回でこの値になる
            {
                eight_turning_mode = nokolat2024::main_control::EIGHT_TURNING_MODE::NEUTRAL_POSITION_R;
                reset_rotation_count(0.75);
                recover_start_time_ = ros_clock->now(); // 水平状態に戻り始める時間を取得
                RCLCPP_INFO(this->get_logger(), "RIGHT END");
                roll_align = false;
            }
        }

        if (eight_turning_mode == nokolat2024::main_control::EIGHT_TURNING_MODE::NEUTRAL_POSITION_L)
        {
            rclcpp::Duration diff = ros_clock->now() - recover_start_time_;

            // 水平の基準状態に戻す
            throttle = throttle_control(eight_turning_target.throttle_target);
            elevator = elevator_control(eight_turning_target.altitude_target, eight_turning_gain.pitch_gain, eight_turning_target.pitch_target_recover + (eight_turning_target.pitch_target - eight_turning_target.pitch_target_recover) * linearDecrease(diff.seconds(), eight_turning_delay.delay_rudder), eight_turning_gain.nose_up_pitch_gain, eight_turning_gain.elevator_gain);
            aileron_l = aileron_control_l(0, eight_turning_gain.aileron_gain); // Roll = 0
            aileron_r = aileron_control_r(0, eight_turning_gain.aileron_gain);
            rudder = neutral_position_.rudder + eight_turning_target.rudder_target_l * linearDecrease(diff.seconds(), eight_turning_delay.delay_rudder);

            // 一定時間経過+ロールが基準値になったら右旋回に移行
            if (-roll_convergence_threshold < roll_error && roll_error < roll_convergence_threshold &&
                diff.seconds() > eight_turning_delay.delay_rudder)
            {
                eight_turning_mode = nokolat2024::main_control::EIGHT_TURNING_MODE::RIGHT_TURNING;
                RCLCPP_INFO(this->get_logger(), "RIGHT START");
                roll_align = false;
            }
        }

        if (eight_turning_mode == nokolat2024::main_control::EIGHT_TURNING_MODE::NEUTRAL_POSITION_R)
        {
            rclcpp::Duration diff = ros_clock->now() - recover_start_time_;

            // 水平の基準状態に戻す
            throttle = throttle_control(eight_turning_target.throttle_target);
            elevator = elevator_control(eight_turning_target.altitude_target, eight_turning_gain.pitch_gain, eight_turning_target.pitch_target_recover + (eight_turning_target.pitch_target - eight_turning_target.pitch_target_recover) * linearDecrease(diff.seconds(), eight_turning_delay.delay_rudder), eight_turning_gain.nose_up_pitch_gain, eight_turning_gain.elevator_gain);
            aileron_l = aileron_control_l(0, eight_turning_gain.aileron_gain); // Roll = 0
            aileron_r = aileron_control_r(0, eight_turning_gain.aileron_gain);
            rudder = neutral_position_.rudder + eight_turning_target.rudder_target_l * linearDecrease(diff.seconds(), eight_turning_delay.delay_rudder);

            // 一定時間経過+ロールが基準値になったら左旋回に移行
            if (-roll_convergence_threshold < roll_error && roll_error < roll_convergence_threshold &&
                diff.seconds() > eight_turning_delay.delay_rudder)
            {
                eight_turning_mode = nokolat2024::main_control::EIGHT_TURNING_MODE::LEFT_TURNING;
                RCLCPP_INFO(this->get_logger(), "LEFT START");
                roll_align = false;
            }
        }

        // 投下装置は閉じる
        drop = drop_control(config.drop_min);
    }

    void auto_landing_control()
    {
        rclcpp::Time current_time = ros_clock->now();
        rclcpp::Duration diff = current_time - start_mode_time_;

        // 加速
        if (diff.seconds() < auto_landing_delay.delay_accel)
        {
            throttle = auto_landing_target.takeoff_throttle_target;

            elevator = neutral_position_.elevator;

            aileron_l = neutral_position_.aileron_l;
            aileron_r = neutral_position_.aileron_r;

            // ラダーはyawの誤差に比例
            rudder = neutral_position_.rudder;
        }
        // 離陸~
        else
        {
            // スロットルはじょじょに下げる
            throttle = auto_landing_target.throttle_target + (auto_landing_target.takeoff_throttle_target - auto_landing_target.throttle_target) * linearDecrease(diff.seconds(), auto_landing_delay.delay_decel);

            elevator = elevator_control(auto_landing_target.induction_altitude_target, auto_landing_gain.pitch_gain, auto_landing_target.pitch_target, auto_landing_gain.nose_up_pitch_gain, auto_landing_gain.elevator_gain);

            altitude_error = pose_received_.z - auto_landing_target.induction_altitude_target;
            target_pitch = cutoff_min_max(auto_landing_gain.pitch_gain * altitude_error + auto_landing_target.pitch_target, -M_PI / 8, M_PI / 6);
            pitch_error = pose_received_.pitch - auto_landing_target.pitch_target;

            aileron_l = aileron_control_l(0, auto_landing_gain.aileron_gain);
            aileron_r = aileron_control_r(0, auto_landing_gain.aileron_gain);

            roll_error = pose_received_.roll - auto_landing_target.roll_target;

            rudder = neutral_position_.rudder + auto_landing_gain.rudder_gain * auto_landing_target.induction_yaw_diff;
        }

        if (drop_timing)
        {
            drop = drop_control(config.drop_max);
        }
        else
        {
            drop = drop_control(config.drop_min);
        }

        if (throttle_off_timing)
        {
            // スロットルを徐々に下げる
            if (daccel_start == false)
            {
                daccel_start_time_ = ros_clock->now();
                daccel_start = true;
            }

            rclcpp::Duration diff = current_time - daccel_start_time_;

            throttle = config.throttle_min + (auto_landing_target.takeoff_throttle_target - config.throttle_min) * linearDecrease(diff.seconds(), auto_landing_delay.delay_land);
        }
    }

    void
    pub_command_data()
    {
        // 制御値を送信
        nokolat2024_msg::msg::Command command;
        command.header.stamp = ros_clock->now();
        command.throttle = cutoff_min_max(throttle, config.throttle_min, config.throttle_max);
        command.elevator = cutoff_min_max(elevator, config.elevator_min, config.elevator_max);
        command.aileron_r = cutoff_min_max(aileron_r, config.aileron_min_r, config.aileron_max_r);
        command.aileron_l = cutoff_min_max(aileron_l, config.aileron_min_l, config.aileron_max_l);
        command.rudder = cutoff_min_max(rudder, config.rudder_min, config.rudder_max);
        command.dropping_device = cutoff_min_max(drop, config.drop_min, config.drop_max);
        command_publisher_->publish(command);
    }

    void pub_rotation_rest_data(double rotation_standard)
    {
        std_msgs::msg::Float32 rotation_counter_reset_msg;
        rotation_counter_reset_msg.data = rotation_standard; // この値でカウントアップするように変更、0~1周
        rotation_counter_reset_publisher_->publish(rotation_counter_reset_msg);
    }

    void pub_ui_data()
    {

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

        std_msgs::msg::Float32 roll_target_msg;
        roll_target_msg.data = auto_turning_target.roll_target;
        roll_target_publisher_->publish(roll_target_msg);

        std_msgs::msg::Float32 roll_error_msg;
        roll_error_msg.data = roll_error;
        roll_error_publisher_->publish(roll_error_msg);
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

    double get_target_altitude()
    {
        return std::accumulate(altitude_history_.begin(), altitude_history_.end(), 0.0) / altitude_history_.size();
    }

    double get_target_throttle()
    {
        return std::accumulate(throttle_history_.begin(), throttle_history_.end(), 0.0) / throttle_history_.size();
    }

    double get_target_yaw()
    {
        return std::accumulate(yaw_history_.begin(), yaw_history_.end(), 0.0) / yaw_history_.size();
    }

    double linearIncrease(double x, double threshold)
    {
        if (x <= 0)
        {
            return 0.0;
        }
        else if (x >= threshold)
        {
            return 1.0;
        }
        else
        {
            return x / threshold;
        }
    }

    double linearDecrease(double x, double threshold)
    {
        if (x <= 0)
        {
            return 1.0;
        }
        else if (x >= threshold)
        {
            return 0.0;
        }
        else
        {
            return 1 - x / threshold;
        }
    }

    // 制御情報を格納
    nokolat2024::main_control::ControlInfo_config config;
    double roll_convergence_threshold;
    double altitude_convergence_threshold;

    nokolat2024::main_control::ControlInfo_gain auto_turning_gain;
    nokolat2024::main_control::ControlInfo_target auto_turning_target;
    nokolat2024::main_control::DelayWindow auto_turning_delay;

    nokolat2024::main_control::ControlInfo_gain rise_turning_gain;
    nokolat2024::main_control::ControlInfo_target_rise rise_turning_target;
    nokolat2024::main_control::DelayWindow rise_turning_delay;

    nokolat2024::main_control::ControlInfo_gain eight_turning_gain;
    nokolat2024::main_control::ControlInfo_target_lr eight_turning_target;
    nokolat2024::main_control::DelayWindow eight_turning_delay;

    nokolat2024::main_control::ControlInfo_gain auto_landing_gain;
    nokolat2024::main_control::ControlInfo_target_land auto_landing_target;
    nokolat2024::main_control::DelayWindow auto_landing_delay;

    nokolat2024::main_control::Pose pose_received_;
    nokolat2024::main_control::Command neutral_position_;

    // パラメータを取得
    YAML::Node control_info_config_;

    // モードが変更されたかどうか、今のモードを記録
    std::string control_mode_;
    bool mode_init = true;

    std::deque<double> altitude_history_;
    std::deque<double> throttle_history_;
    std::deque<double> yaw_history_;

    std::shared_ptr<rclcpp::Clock> ros_clock;

    rclcpp::Time start_mode_time_;

    bool roll_align = false;
    rclcpp::Time roll_align_time_;
    rclcpp::Time recover_start_time_;

    double turning_count;

    bool drop_timing = false;
    bool throttle_off_timing = false;
    bool daccel_start = false;
    rclcpp::Time daccel_start_time_;

    nokolat2024::main_control::RISE_TURNING_MODE rise_turning_mode;
    nokolat2024::main_control::EIGHT_TURNING_MODE eight_turning_mode;

    double throttle;
    double altitude_error;
    double target_pitch;
    double pitch_error;
    double elevator;
    double aileron_l;
    double aileron_r;
    double roll_error;
    double yaw_error;
    double rudder;
    double drop;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr neutral_position_subscriber_;

    rclcpp::Subscription<nokolat2024_msg::msg::Command>::SharedPtr command_explicit_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscriber_;
    rclcpp::Subscription<nokolat2024_msg::msg::Rpy>::SharedPtr rpy_subscriber_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rotation_counter_subscriber_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drop_timing_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr throttle_off_timing_subscriber_;

    rclcpp::Publisher<nokolat2024_msg::msg::Command>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rotation_counter_reset_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_target_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainControlNode>());
    rclcpp::shutdown();
    return 0;
}