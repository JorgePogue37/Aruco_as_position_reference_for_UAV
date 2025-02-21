cmake_minimum_required(VERSION 3.10)  # Or your minimum required CMake version
project(aruco_pose_publisher)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find TF2 (the exact name might vary: tf2, tf2_ros, etc.)
find_package(tf2_ros REQUIRED) # Try tf2 if tf2_ros fails
find_package(tf2_ros REQUIRED)

include_directories(
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

link_directories(
    /opt/ros/noetic/lib  # Adjust if the libraries are in a different location
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find other ROS dependencies (roscpp, geometry_msgs, std_msgs) - VERY IMPORTANT
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(aruco_pose_publisher src/aruco_pose_publisher.cpp) # Add src/ if needed

# Link libraries (MANUALLY - this is the tricky part)
target_link_libraries(aruco_pose_publisher
    tf2 tf2_ros
    ${Eigen3_LIBRARIES} # From find_package(Eigen3)
    ${roscpp_LIBRARIES} # From find_package(roscpp)
    ${geometry_msgs_LIBRARIES} # From find_package(geometry_msgs)
    ${std_msgs_LIBRARIES} # From find_package(std_msgs)
    pthread # Likely needed
)

# Include directories (MANUALLY)
include_directories(
    ${TF2_INCLUDE_DIRS} # From find_package(tf2_ros) or find_package(tf2)
    ${Eigen3_INCLUDE_DIRS} # From find_package(Eigen3)
    ${roscpp_INCLUDE_DIRS} # From find_package(roscpp)
    ${geometry_msgs_INCLUDE_DIRS} # From find_package(geometry_msgs)
    ${std_msgs_INCLUDE_DIRS} # From find_package(std_msgs)
)
