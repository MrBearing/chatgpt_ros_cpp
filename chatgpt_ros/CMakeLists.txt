cmake_minimum_required(VERSION 3.8)
project(chatgpt_ros)

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TARGET chat_gpt_action_server)
ament_auto_add_library(
  ${TARGET}
    SHARED
      src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "chatgpt_ros::ChatGptServer"
    EXECUTABLE ${TARGET}_exec)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)