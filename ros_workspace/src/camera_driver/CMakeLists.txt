cmake_minimum_required(VERSION 3.22)
project(camera_driver)

# By default use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_msgs REQUIRED)

include_directories(/usr/include)
link_directories(/usr/lib/x86_64-linux-gnu)

# Find MVSDK library
find_library(MVSDK_LIB
        NAMES MVSDK
        PATHS /lib
        NO_DEFAULT_PATH)

# publisher executable
add_executable(publisher src/publisher.cpp
        include/camera_driver/DeviceTime.hpp
        include/camera_driver/TimeAlign.hpp
        include/camera_driver/jsonConfig.hpp)
ament_target_dependencies(publisher rclcpp std_msgs my_msgs)
target_link_libraries(publisher ${MVSDK_LIB} lz4)
target_include_directories(publisher PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# subscriber executable
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs my_msgs)
target_link_libraries(subscriber ${MVSDK_LIB})
target_include_directories(subscriber PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# post_processing executable
add_executable(post_processing src/post_processing.cpp)
ament_target_dependencies(post_processing rclcpp std_msgs my_msgs)
target_link_libraries(post_processing ${MVSDK_LIB})
target_include_directories(post_processing PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# install launch file
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch  
)

install(TARGETS
        publisher
        subscriber
        post_processing
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
