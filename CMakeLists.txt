cmake_minimum_required(VERSION 3.14)
project(listener)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common io)
find_package (OpenCV REQUIRED)
find_package (Boost REQUIRED)
find_package(image_transport REQUIRED)  # Required for image transport functionality
find_package(cv_bridge REQUIRED)  # Required for OpenCV to ROS image conversion
find_package(sensor_msgs REQUIRED)  # Since you're subscribing to an IMU topic




# listener
ament_auto_add_library(listener SHARED
        src/listener_node.cpp
        )


# Add compile definitions and include directories for PCL and OpenCV
target_compile_definitions(listener PUBLIC ${PCL_DEFINITIONS})
target_include_directories(listener PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# Link PCL and OpenCV libraries
target_link_libraries(listener ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})




# Register the listener node
rclcpp_components_register_node(listener
        PLUGIN "Listener"
        EXECUTABLE listener_node
        )


######

ament_auto_package(INSTALL_TO_SHARE
        launch
        )

