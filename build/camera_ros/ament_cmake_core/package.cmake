set(_AMENT_PACKAGE_NAME "camera_ros")
set(camera_ros_VERSION "0.2.0")
set(camera_ros_MAINTAINER "Christian Rauch <Rauch.Christian@gmx.de>")
set(camera_ros_BUILD_DEPENDS "libcamera" "rclcpp" "rclcpp_components" "sensor_msgs" "camera_info_manager" "cv_bridge")
set(camera_ros_BUILDTOOL_DEPENDS "ament_cmake")
set(camera_ros_BUILD_EXPORT_DEPENDS "libcamera" "rclcpp" "rclcpp_components" "sensor_msgs" "camera_info_manager" "cv_bridge")
set(camera_ros_BUILDTOOL_EXPORT_DEPENDS )
set(camera_ros_EXEC_DEPENDS "ros2launch" "image_view" "libcamera" "rclcpp" "rclcpp_components" "sensor_msgs" "camera_info_manager" "cv_bridge")
set(camera_ros_TEST_DEPENDS "ament_lint_auto" "ament_cmake_clang_format" "ament_cmake_cppcheck" "ament_cmake_flake8" "ament_cmake_lint_cmake" "ament_cmake_mypy" "ament_cmake_pep257" "ament_cmake_pyflakes" "ament_cmake_xmllint")
set(camera_ros_GROUP_DEPENDS )
set(camera_ros_MEMBER_OF_GROUPS )
set(camera_ros_DEPRECATED "")
set(camera_ros_EXPORT_TAGS)
list(APPEND camera_ros_EXPORT_TAGS "<build_type>ament_cmake</build_type>")