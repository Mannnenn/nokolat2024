cmake_minimum_required(VERSION 3.8)
project(nokolat2024)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(message_filters REQUIRED)
find_package(nokolat2024_msg REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)


install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)


find_package(rosidl_default_generators REQUIRED)


add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher rclcpp tf2_ros geometry_msgs tf2_geometry_msgs)

add_executable(image_diff_extraction src/image_diff_extraction.cpp)
ament_target_dependencies(image_diff_extraction rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(depth_estimation src/depth_estimation.cpp)
ament_target_dependencies(depth_estimation rclcpp geometry_msgs yaml-cpp)
target_link_libraries(depth_estimation yaml-cpp)

add_executable(position_estimation src/position_estimation.cpp)
ament_target_dependencies(position_estimation rclcpp image_transport sensor_msgs cv_bridge OpenCV geometry_msgs yaml-cpp)
target_link_libraries(position_estimation yaml-cpp)

add_executable(point2pointStamped src/point2pointStamped.cpp)
ament_target_dependencies(point2pointStamped rclcpp geometry_msgs nav_msgs)

add_executable(cog_corrected src/cog_corrected.cpp)
ament_target_dependencies(cog_corrected rclcpp sensor_msgs geometry_msgs image_transport cv_bridge OpenCV yaml-cpp)
target_link_libraries(cog_corrected yaml-cpp)

add_executable(pub_command src/pub_command.cpp)
ament_target_dependencies(pub_command rclcpp sensor_msgs std_msgs)

add_executable(image_sync_node src/timestamp_sync.cpp)
ament_target_dependencies(image_sync_node rclcpp sensor_msgs message_filters)

add_executable(projection_xy src/projection_xy.cpp)
ament_target_dependencies(projection_xy   rclcpp  geometry_msgs tf2_ros)

add_executable(recive_command src/recive_command.cpp)
ament_target_dependencies(recive_command rclcpp  geometry_msgs nokolat2024_msg)

add_executable(tf2rpy src/tf2rpy.cpp)
ament_target_dependencies(tf2rpy rclcpp  geometry_msgs std_msgs tf2_ros nokolat2024_msg)

add_executable(imu_tof2posAndQuat src/imu_tof2posAndQuat.cpp)
ament_target_dependencies(imu_tof2posAndQuat rclcpp  geometry_msgs tf2_ros)

add_executable(main_control src/main_control.cpp)
ament_target_dependencies(main_control rclcpp  std_msgs geometry_msgs tf2_ros nokolat2024_msg tf2_geometry_msgs)
target_link_libraries(main_control yaml-cpp)

add_executable(get_neutral_position src/get_neutral_position.cpp)
ament_target_dependencies(get_neutral_position rclcpp nokolat2024_msg)

add_executable(send_command src/send_command.cpp)
ament_target_dependencies(send_command rclcpp nokolat2024_msg geometry_msgs)

add_executable(path_generator src/path_generator.cpp)
ament_target_dependencies(path_generator rclcpp geometry_msgs tf2_ros)
target_link_libraries(path_generator Eigen3::Eigen)


add_executable(path_followers src/path_followers.cpp)
ament_target_dependencies(path_followers rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs nokolat2024_msg)

add_executable(rotation_counter src/rotation_counter.cpp)
ament_target_dependencies(rotation_counter rclcpp std_msgs nokolat2024_msg)

add_executable(pub_ui_command src/pub_ui_command.cpp)
ament_target_dependencies(pub_ui_command rclcpp std_msgs geometry_msgs tf2_ros nokolat2024_msg tf2_geometry_msgs)

add_executable(pub_vel src/pub_vel.cpp)
ament_target_dependencies(pub_vel rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(pub_vel_link src/pub_vel_link.cpp)
ament_target_dependencies(pub_vel_link rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(pub_cameraInfo src/pub_cameraInfo.cpp)
ament_target_dependencies(pub_cameraInfo rclcpp sensor_msgs geometry_msgs)

add_executable(timer src/timer.cpp)
ament_target_dependencies(timer rclcpp std_msgs)

add_executable(path_test src/path_test.cpp)
ament_target_dependencies(path_test rclcpp tf2 tf2_ros geometry_msgs sensor_msgs)

add_executable(pub_timing src/pub_timing.cpp)
ament_target_dependencies(pub_timing rclcpp tf2 tf2_ros std_msgs geometry_msgs)

add_executable(pub_area src/pub_area.cpp)
ament_target_dependencies(pub_area rclcpp geometry_msgs)

add_executable(camera_imu_calib src/camera_imu_calib.cpp)
ament_target_dependencies(camera_imu_calib rclcpp sensor_msgs)

install(TARGETS
  tf_publisher
  image_diff_extraction
  depth_estimation
  position_estimation
  point2pointStamped
  pub_vel
  pub_vel_link
  cog_corrected
  pub_command
  image_sync_node
  projection_xy
  recive_command
  tf2rpy
  imu_tof2posAndQuat
  main_control
  get_neutral_position
  send_command
  path_generator
  path_followers
  rotation_counter
  pub_ui_command
  pub_cameraInfo
  timer
  path_test
  pub_timing
  pub_area
  camera_imu_calib
  DESTINATION lib/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()