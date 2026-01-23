cmake_minimum_required(VERSION 3.8)
project(line_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Dependencies =====
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# For OpenCV bridge (ROS Image <-> cv::Mat)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV itself (system library)
find_package(OpenCV REQUIRED)

# ===== Targets =====
add_executable(line_detector_node src/line_detector_node.cpp)
ament_target_dependencies(line_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  # image_transport
)
target_link_libraries(line_detector_node ${OpenCV_LIBS})

add_executable(line_controller_node src/line_controller_node.cpp)
ament_target_dependencies(line_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(camera_node ${OpenCV_LIBS})

# ===== Install =====
install(TARGETS
  line_detector_node
  line_controller_node
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ===== Testing =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
