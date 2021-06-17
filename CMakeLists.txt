cmake_minimum_required(VERSION 2.8.2)
project(wgs_conversions)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# enable C++11 support
#set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++0x")
#set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
# set(CMAKE_CXX_FLAGS "-std=gnu++11")
#set(CMAKE_CXX_FLAGS "-std=c++0x")


add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)

find_package(catkin COMPONENTS
    eigen_conversions
    geographic_msgs
    geometry_msgs
    message_filters
    message_generation
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    xmlrpcpp
    swri_math_util
    swri_transform_util
    swri_yaml_util
    )

generate_messages()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES wgs_conversions
    )

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a cpp library
#add_library(wgs_conversions
#    src/wgs_conversions.cpp
#    include/wgs_conversions.h)
#add_dependencies(wgs_conversions libGeographiccc)
#target_link_libraries(wgs_conversions ${catkin_LIBRARIES} libGeographiccc)


# add_exec
#add_executable(wgs_convertor_node src/wgs_convertor_node.cpp src/wgs_convertor.cpp)
#add_dependencies(wgs_convertor_node libGeographiccc)
#target_link_libraries(wgs_convertor_node wgs_conversions ${catkin_LIBRARIES})

add_executable(wgs_convertor_node src/wgs_convertor_node.cpp src/swri_ins_transform.cpp
    src/tf_lisener.cpp src/tf_broadcaster.cpp)
#add_dependencies(wgs_convertor_node libGeographiccc)
target_link_libraries(wgs_convertor_node ${catkin_LIBRARIES})
