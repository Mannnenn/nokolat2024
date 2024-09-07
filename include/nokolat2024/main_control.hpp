#include <rclcpp/rclcpp.hpp>

#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <deque>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nokolat2024_msg/msg/rpy.hpp"
#include "nokolat2024_msg/msg/command.hpp"

#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

namespace nokolat2024
{
    namespace main_control
    {
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
            double throttle_target;
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
            double yaw;
            double z;
        };

        struct DelayWindow
        {
            uint delay_rudder;
            uint delay_rudder_counter;
        };
    } // namespace main_control
} // namespace nokolat2024

#endif // MAIN_CONTROL_HPP