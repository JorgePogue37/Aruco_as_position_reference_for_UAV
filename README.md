# Aruco-based Position Reference for UAV

This project provides a position reference for a Unmanned Aerial Vehicle (UAV) using an Aruco marker and a camera.  It consists of three modules:

1. **Aruco Detection and Data Transmission:** This module captures images from a camera, detects an Aruco marker, calculates its translation and rotation vectors/matrix, and transmits this data along with the captured image via UDP.

2. **Position Estimation and MAVROS Publication:** This module receives the Aruco marker data and image via UDP, calculates the UAV's position and orientation based on the received information, and publishes the estimated position using MAVROS (MAVlink ROS).

3. **Real-time Visualization:** This module receives the camera image via UDP and displays it in real-time, allowing visualization of the camera's view.

## Overview

The system uses an Aruco marker as a ground truth reference for the UAV's position.  The camera mounted on the UAV captures images, and the Aruco detection module extracts the marker's pose.  This information is then used to estimate the UAV's position relative to the marker.  The estimated position is published using MAVROS, enabling the UAV's flight controller to use this information for navigation or control.  The real-time visualization module provides a visual feedback of the camera's perspective.

## Modules

### 1. Aruco Detection and Data Transmission

* **Description:** This module is responsible for capturing images from the camera, detecting the Aruco marker, and calculating its pose (translation and rotation).  It then transmits this pose data and the raw camera image over UDP.
* **Dependencies:**
    * OpenCV (for image processing and Aruco detection)
    * [Add other dependencies, e.g., for camera capture, UDP communication]
* **Usage:**
    * [Provide instructions on how to run this module, including command-line arguments, configuration files, etc.]
    * Example: `python aruco_detector.py --camera_id 0 --udp_port 5000`

### 2. Position Estimation and MAVROS Publication

* **Description:** This module receives the Aruco pose data (translation and rotation vectors/matrix) via UDP, processes the data to estimate the UAV's position and orientation, and publishes the estimated position using MAVROS. 
* **Dependencies:**
    * ROS (Robot Operating System)
    * MAVROS
    * [Add other dependencies, e.g., for UDP communication, position estimation libraries]
* **Usage:**
    * [Provide instructions on how to run this module within ROS, including launch files, ROS parameters, etc.]
    * Example: `roslaunch aruco_position_estimator aruco_position_estimator.launch`

### 3. Real-time Visualization

* **Description:** This module receives the camera image via UDP and displays it in a window, providing real-time visual feedback.
* **Dependencies:**
    * OpenCV
    * [Add other dependencies, e.g., for UDP communication]
* **Usage:**
    * [Provide instructions on how to run this module, including command-line arguments, configuration files, etc.]
    * Example: `python image_viewer.py --udp_port 5001`

## Installation

[Provide detailed installation instructions, including how to install the dependencies for each module.  If using ROS, explain how to set up a workspace and build the packages.]

## Usage

[Provide a comprehensive guide on how to run the system, including how to start each module, configure the parameters (e.g., camera ID, UDP ports, Aruco marker size), and any other relevant information.]

## Configuration

[Describe the configuration options for each module, such as:
* Camera parameters (e.g., camera ID, resolution)
* UDP ports
* Aruco marker size
* ROS parameters (if applicable)]

## Contributing

[Explain how others can contribute to the project.]

## License

[Specify the license under which the code is distributed (e.g., MIT, GPLv3).]

## Acknowledgements

[Acknowledge any third-party libraries or tools used in the project.]
