cmake_minimum_required(VERSION 3.8)
project(test_joint_state_fetch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Generate custom ROS messages
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PrintPose.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

#########
# Add the executable
add_executable(test_joint_state src/test_joint_state_fetch.cpp)

# Set target properties for C99 and C++17
target_compile_features(test_joint_state PUBLIC c_std_99 cxx_std_17)
rosidl_target_interfaces(test_joint_state ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Include generated headers
# Note: ${PROJECT_NAME}__rosidl_typesupport_cpp might need to be replaced based on your exact build environment and interface generation settings
target_include_directories(test_joint_state PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# Add dependencies to the executable
ament_target_dependencies(
  test_joint_state
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"  # Add geometry_msgs if your service or application requires it
)

# Install the executable
install(TARGETS test_joint_state
  DESTINATION lib/${PROJECT_NAME})

# Linter settings
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # You can uncomment the below lines to enable specific linters once your project is ready
  # set(ament_cmake_cpplint_FOUND TRUE)
  # set(ament_cmake_copyright_FOUND TRUE)
endif()

# Package configuration
ament_package()
