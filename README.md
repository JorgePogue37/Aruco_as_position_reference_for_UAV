# Aruco-based Position Reference for UAV

This project provides a position reference for a Unmanned Aerial Vehicle (UAV) using an Aruco marker and a camera. It consists of three modules:

1. **Aruco Detection and Data Transmission:** This module captures images from a camera, detects an Aruco marker, calculates its translation and rotation vectors/matrix, and transmits this data along with the captured image via UDP.
2. **Position Estimation and MAVROS Publication:** This module receives the Aruco marker data and image via UDP, calculates the UAV's position and orientation based on the received information, and publishes the estimated position using MAVROS (MAVlink ROS).
3. **Real-time Visualization:** This module receives the camera image via UDP and displays it in real-time, allowing visualization of the camera's view.

## Overview

The system uses an Aruco marker as a ground truth reference for the UAV's position.  The camera mounted on the UAV captures images, and the Aruco detection module extracts the marker's pose.  This information is then used to estimate the UAV's position relative to the marker.  The estimated position is published using MAVROS, enabling the UAV's flight controller to use this information for navigation or control.  The real-time visualization module provides a visual feedback of the camera's perspective.

## Dependencies

* **General:**
    * OpenCV - See the official OpenCV installation guide: [https://docs.opencv.org/4.10.0/d7/d9f/tutorial_linux_install.html](https://docs.opencv.org/4.10.0/d7/d9f/tutorial_linux_install.html)
* **ROS (for Position Estimation):**
    * ROS (Robot Operating System) - Follow the ROS installation instructions: [http://www.ros.org/](http://www.ros.org/))
    * MAVROS - Install the required MAVROS packages.
    * `tf` and `tf2` packages
    * `geometry_msgs`


## Installation

1. **System Dependencies:**
    * **OpenCV:** Follow the instructions in the official OpenCV installation guide linked above.  
    * **ROS and related packages:** Install ROS, MAVROS, `tf`, `tf2`, and `geometry_msgs` on the computer that will run the `position_estimation` node. Example (using apt):
        ```bash
        sudo apt-get install ros-<your_ros_distro>-desktop-full ros-<your_ros_distro>-mavros ros-<your_ros_distro>-mavros-extras ros-<your_ros_distro>-tf ros-<your_ros_distro>-tf2-ros ros-<your_ros_distro>-geometry-msgs
        ```
        (Replace `<your_ros_distro>` with your ROS distribution, e.g., `melodic`, `noetic`, `kinetic`, `humble`, etc.)

2. **Build the Project:**
    * Clone the repository.
    * Inside of the folder of each module create a build directory: `mkdir build`
    * Navigate to the build directory: `cd build`
    * Run CMake: `cmake ..`
    * Build: `make`  (This will build the executables for each module separately.)

## Usage

### Important: 

The `Aruco_Parcel_Module` and `aruco_pose_publisher` modules are intended to run on the same computer (e.g., on the drone's onboard computer). The `VisualFeedback_Display` module runs on a *separate* computer for visualization.

Make sure to modify the image receiver computer IP and the size of the aruco marker on the Main.cpp file of the Aruco_Parcel_Marker module and the camera parameters on the CameraCalibration.yaml file. [https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

1. **On the Drone (or Main Computer):**
    * **ROS Core (if applicable):** `rosmaster` (if using ROS for the position estimation module)
    * **MAVROS:** Launch MAVROS:
        
        #### For ArduPilot:
        ```bash
        roslaunch mavros apm.launch  # Or your specific MAVROS launch file
        ```
        #### For PX4:
        ```bash
        roslaunch mavros px4.launch  # Or your specific MAVVROS launch file
        ```
    * **Aruco Detection:**
      
        Inside of /build/Main folder of Aruco_Parcel_Marker
        ```bash
        ./Aruco_Parcel_Module <camera_id> <aruco_id> <video_flag>
        ```
        * **Camera:** `<camera_id>` id of the video device on the system.
        * **Aruco ID:** `<aruco_id>`
        * **Video Flag:** `<video_flag>` 1 for video feedback, 0 to disable it.
    * **Position Estimation (if using ROS):**
      
        Inside of /build folder of Aruco_pose_publisher
        ```bash
        ./aruco_pose_publisher
        ```

2. **On the Visualization Computer:**
    * **Real-time Visualization:**

        Inside of /build folder of VisualFeedback_Display
        ```bash
        ./VisualFeedback_Display <UDP_PORT>
        ```
        * **UDP:**  Ports are configured via command-line arguments. Make sure the ports are consistent across modules. This port has to match the one stablished on the Main.cpp file of the Aruco_Parcel_Marker module, default: 28001
## NOTE:
After launching these modules the drone should be receiving position correctly, but for it to be able to hold it you may have to modify some parameters of the Autopilot. With programs like Qground you can do this in real time.
