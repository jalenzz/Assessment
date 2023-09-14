cmake_minimum_required(VERSION 3.8)
project(task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${OpenCV_LIBS})
ament_target_dependencies(camera_node rclcpp OpenCV std_msgs my_interfaces cv_bridge image_transport)

add_executable(find_red_node src/find_red_node.cpp)
target_link_libraries(find_red_node ${OpenCV_LIBS})
ament_target_dependencies(find_red_node rclcpp OpenCV std_msgs my_interfaces cv_bridge)

install(TARGETS
    camera_node
    find_red_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
