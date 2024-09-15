#include <rclcpp/rclcpp.hpp>

#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <deque>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nokolat2024_msg/msg/rpy.hpp"
#include "nokolat2024_msg/msg/command.hpp"

#include "std_msgs/msg/float32.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

namespace nokolat2024
{
    namespace main_control
    {
        const bool TRANSFER = 0;
        const bool RECOVER = 1;

        // Define control mode
        enum CONTROL_MODE
        {
            MANUAL = 0,
            AUTO_TURNING = 1,
            AUTO_RISE_TURNING = 2,
            AUTO_EIGHT_TURNING = 3,
            AUTO_LANDING = 4,
        };

        enum EIGHT_TURNING_MODE
        {
            LEFT_TURNING = 0,
            RIGHT_TURNING = 1,
            NEUTRAL_POSITION_L = 2,
            NEUTRAL_POSITION_R = 3,
        };

        // マップを初期化
        const std::unordered_map<int16_t, std::string> control_mode_map = {
            {CONTROL_MODE::MANUAL, "MANUAL"},
            {CONTROL_MODE::AUTO_TURNING, "AUTO_TURNING"},
            {CONTROL_MODE::AUTO_RISE_TURNING, "AUTO_RISE_TURNING"},
            {CONTROL_MODE::AUTO_EIGHT_TURNING, "AUTO_EIGHT_TURNING"},
            {CONTROL_MODE::AUTO_LANDING, "AUTO_LANDING"},
        };

        struct ControlInfo_config
        {
            double throttle_max;
            double throttle_min;
            double elevator_max;
            double elevator_min;
            double aileron_max_r;
            double aileron_min_r;
            double aileron_max_l;
            double aileron_min_l;
            double rudder_max;
            double rudder_min;
            double drop_max;
            double drop_center;
            double drop_min;
        };

        struct ControlInfo_gain
        {
            double throttle_gain;
            double elevator_gain;
            double rudder_gain;
            double aileron_gain;
            double pitch_gain;
            double nose_up_pitch_gain;
        };

        struct ControlInfo_target
        {
            double velocity_target;
            double altitude_target;
            double throttle_target;
            double roll_target;
            double pitch_target;
            double rudder_target;
        };

        struct ControlInfo_target_rise : public ControlInfo_target
        {
            double lower_altitude_target;
            double higher_altitude_target;
            double rise_throttle_target;
            double rise_roll_target;
            double rise_pitch_target;
            double rise_rudder_target;
        };

        struct ControlInfo_target_lr : public ControlInfo_target
        {
            double roll_target_l;
            double roll_target_r;
            double rudder_target_l;
            double rudder_target_r;
            double pitch_target_l;
            double pitch_target_r;
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
            double yaw;
            double z;
        };

        struct DelayWindow
        {
            double delay_rudder;
            double delay_accel;
            double delay_decel;
            double delay_rise;
        };
    } // namespace main_control
} // namespace nokolat2024

#endif // MAIN_CONTROL_HPP