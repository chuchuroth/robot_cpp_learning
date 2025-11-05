
cmake_minimum_required(VERSION 3.5)
project(robot_cpp_learning)

# 默认find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 新增
find_package(cv_bridge REQUIRED)    # 新增
find_package(image_transport REQUIRED)  # 新增
find_package(OpenCV REQUIRED)       # 新增OpenCV

# 硬件条件（CMake时定义：-D RASPI=ON 或 -D JETSON=ON）
option(RASPI "Build for Raspberry Pi" OFF)
option(JETSON "Build for Jetson" OFF)
if(RASPI)
    find_package(wiringPi REQUIRED)  # Pi库
    add_definitions(-D__RASPBERRY_PI__)
elseif(JETSON)
    find_package(Jetson.GPIO REQUIRED)  # Jetson库
    add_definitions(-D__JETSON__)
endif()

# 可执行文件
add_executable(motor_class src/motor_class.cpp)
target_include_directories(motor_class PRIVATE src/)
target_link_libraries(motor_class ${OpenCV_LIBS})  # 如需

add_executable(sensor_interface src/sensor_interface.cpp)
add_executable(camera_sensor src/camera_sensor.cpp)
add_executable(thread_sensor_sim src/thread_sensor_sim.cpp)

# ROS2节点
add_executable(ros2_motor_node src/ros2_motor_node.cpp)
ament_target_dependencies(ros2_motor_node rclcpp std_msgs)
target_include_directories(ros2_motor_node PRIVATE src/)
if(RASPI)
    target_link_libraries(ros2_motor_node wiringPi)
endif()

add_executable(ros2_camera_node src/ros2_camera_node.cpp)
ament_target_dependencies(ros2_camera_node rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(ros2_camera_node ${OpenCV_LIBS})

add_executable(ros2_image_process_node src/ros2_image_process_node.cpp)
ament_target_dependencies(ros2_image_process_node rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(ros2_image_process_node ${OpenCV_LIBS})

install(TARGETS
  motor_class sensor_interface camera_sensor thread_sensor_sim
  ros2_motor_node ros2_camera_node ros2_image_process_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()