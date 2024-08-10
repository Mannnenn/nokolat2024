cmake_minimum_required(VERSION 3.8)
project(nokolat2024)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(imutof REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport)
find_package(geometry_msgs)
find_package(OpenCV)

find_package(OpenMP REQUIRED)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

find_package(rosidl_default_generators REQUIRED)



add_executable(imu2tf src/imu2tf.cpp)
ament_target_dependencies(imu2tf rclcpp tf2_ros sensor_msgs geometry_msgs std_msgs)


add_executable(infra2bivaluation src/infra2bivaluation.cpp)
ament_target_dependencies(infra2bivaluation rclcpp sensor_msgs cv_bridge image_transport OpenCV)

add_executable(image_diff_extraction src/image_diff_extraction.cpp)
ament_target_dependencies(image_diff_extraction rclcpp sensor_msgs cv_bridge OpenCV OpenMP)

add_executable(stereo_maching src/stereo_maching.cpp)
ament_target_dependencies(stereo_maching rclcpp image_transport sensor_msgs cv_bridge OpenCV)

add_executable(depth_estimation src/depth_estimation.cpp)
ament_target_dependencies(depth_estimation rclcpp image_transport sensor_msgs cv_bridge OpenCV)

add_executable(position_estimation src/position_estimation.cpp)
ament_target_dependencies(position_estimation rclcpp image_transport sensor_msgs cv_bridge OpenCV geometry_msgs)

install(TARGETS
  imu2tf
  infra2bivaluation
  image_diff_extraction
  stereo_maching
  depth_estimation
  position_estimation
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