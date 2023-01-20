cmake_minimum_required(VERSION 2.8.3)
project(slam_img_analyzer)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    rospy
    message_generation
    genmsg
    message_filters
    geometry_msgs
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

## Declare ROS messages and services
## Add messages
add_message_files(
    FILES
    SlamPoseStamped.msg
    StringStamped.msg
)

## Generate added messages and services
generate_messages(
    DEPENDENCIES 
    std_msgs
    geometry_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    
)

# Build cpp files
include_directories(
    include ${catkin_INCLUDE_DIRS}
    SYSTEM ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Create executables
add_executable(decision_maker src/decision_node.cpp src/decision_maker.cpp)
target_link_libraries(decision_maker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(decision_maker slam_img_analyzer)

add_executable(example_publisher src/example_publisher.cpp)
target_link_libraries(example_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(example_publisher slam_img_analyzer)