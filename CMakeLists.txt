cmake_minimum_required(VERSION 3.5)
project(paint_cloud)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()


find_package(ament_lint_auto REQUIRED) 
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(painting_services REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

#find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
#find_package(rosidl_generator_cpp REQUIRED)
#find_package(rosidl_typesupport_cpp REQUIRED)
#find_package(rosidl_typesupport_introspection_cpp REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
#find_package(example_interfaces REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_executable(paint_cloud_server src/paint_cloud_server.cpp src/paint_cloud_component.cpp ${EIGEN3_LIBRARIES} ${PCL_LIBRARY_DIRS})
ament_target_dependencies(paint_cloud_server rclcpp painting_services OpenCV std_msgs sensor_msgs image_transport Eigen3 PCL tf2 tf2_ros tf2_sensor_msgs)
#target_include_directories(paint_cloud_server ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(paint_rosbag src/paint_rosbag.cpp)
ament_target_dependencies(paint_rosbag rclcpp rosbag2_cpp rosbag2_storage sensor_msgs OpenCV painting_services)

ament_auto_add_library(paint_cloud_component SHARED src/paint_cloud_component.cpp)
target_link_libraries(paint_cloud_component
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

install(TARGETS
  paint_cloud_server
  paint_rosbag
  DESTINATION lib/${PROJECT_NAME})

rclcpp_components_register_node(paint_cloud_component
  PLUGIN "PaintCloudComponent"
  EXECUTABLE paint_cloud_node
)

ament_auto_package(INSTALL_TO_SHARE
    launch
    config
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir==='${dir}'")
endforeach()

