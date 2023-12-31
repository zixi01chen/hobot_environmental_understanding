cmake_minimum_required(VERSION 3.5)
project(sense_position)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hobot_autonomous_moving_msgs REQUIRED)

include_directories(include
  ${PROJECT_SOURCE_DIR}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib 
  thirdparty/${PREFIX_PATH}/lib
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/sense_position_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  geometry_msgs
  ai_msgs
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  hobot_autonomous_moving_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
