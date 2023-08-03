cmake_minimum_required(VERSION 3.8)
project(diff_drive)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)


# Add the include directories to the include paths
include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(/opt/ros/humble/include/tf2)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(include ${EIGEN3_INCLUDE_DIR})
include_directories(/usr/include/i2c)

# Create library targets
add_library(Robot_Motion src/Robot_Motion.cpp)
add_library(EKFNode src/EKFNode.cpp)
add_library(ControllerNode src/ControllerNode.cpp)
add_library(Imu_Reader src/Imu_Reader.cpp)
add_library(Odom_Reader src/Odom_Reader.cpp)

# Set target include directories
target_include_directories(Robot_Motion PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  /usr/include/i2c
)

target_include_directories(EKFNode PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS} 
  ${sensor_msgs_INCLUDE_DIRS} 
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS} 
  ${tf2_ros_INCLUDE_DIRS}
)
target_include_directories(ControllerNode PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} 
  ${tf2_ros_INCLUDE_DIRS}
)
target_include_directories(Imu_Reader PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS} 
  ${tf2_ros_INCLUDE_DIRS}
  /usr/include/i2c
)
target_include_directories(Odom_Reader PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS}
  /usr/include/i2c
)

# Link library targets with required dependencies
target_link_libraries(Robot_Motion PRIVATE /usr/lib/x86_64-linux-gnu/libi2c.so)
target_link_libraries(ControllerNode PUBLIC ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES})
target_link_libraries(Imu_Reader PRIVATE ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${tf2_ros_LIBRARIES} Eigen3::Eigen /usr/lib/x86_64-linux-gnu/libi2c.so)
target_link_libraries(Odom_Reader PRIVATE ${rclcpp_LIBRARIES} ${nav_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} /usr/lib/x86_64-linux-gnu/libi2c.so)
target_link_libraries(EKFNode PUBLIC ${rclcpp_LIBRARIES} ${nav_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${tf2_ros_LIBRARIES} Eigen3::Eigen)

# Create an executable target for main.cpp
add_executable(diff_drive src/main.cpp)

# Link the executable with the necessary libraries
target_link_libraries(diff_drive PRIVATE EKFNode ControllerNode Imu_Reader Odom_Reader Robot_Motion)
# ament_target_dependencies(diff_drive rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros Eigen3)

# Install the main executable
install(TARGETS
  diff_drive
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library targets
install(TARGETS
  Robot_Motion
  EKFNode
  ControllerNode
  Imu_Reader
  Odom_Reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the header files
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_package()
