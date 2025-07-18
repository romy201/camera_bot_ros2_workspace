# camera_bot_ros2_workspace

A ROS 2 project for real-time face detection from a webcam feed, from which an action is published to a topic.
## Description

This repository contains a ROS 2 workspace designed to demonstrate a basic human-robot interaction using real-time face detection using python. 
It processes a live video stream from a webcam, identifies the largest face in the frame, and calculates its horizontal deviation from the image center. Based on this deviation, it publishes angular velocity commands (`Twist` messages) that a hypothetical robot could use to turn and face the person. 
Additionally, for external monitoring or further processing, it publishes detailed bounding box and center coordinates of the detected face using a custom message type.

## Features

* **Real-time Face Detection:** Utilizes OpenCV's Haar Cascades (`haarcascade_frontalface_default.xml`) for face detection from a live webcam feed.
* **Angular Robot Control:** Publishes `geometry_msgs/msg/Twist` commands on the `/robotAction` topic to control the robot's left/right turn to keep the detected face centered in the camera frame.
* **Detailed Detection Information:** Publishes `my_robot_interfaces/msg/DetectionInfo` custom messages on the `/robotDetection/detection_info` topic, providing raw bounding box, center coordinates, and a flag indicating detection status.
* **Live Visual Feedback:** Displays the processed webcam feed with bounding boxes and center points drawn on detected faces.

## How It Works

The project operates through two main ROS 2 nodes:

1.  **`webcam_publisher` node (from `camera_bot_pkg`):**
    * Captures live video frames from the local webcam.
    * Publishes these frames as `sensor_msgs/msg/Image` messages on the `/camera/image_raw` topic.

2.  **`human_detector` node (from `camera_bot_pkg`):**
    * Subscribes to the `/camera/image_raw` topic.
    * Converts the ROS `Image` messages to OpenCV format using `cv_bridge`.
    * Applies a Haar Cascade classifier to detect faces in the grayscale image.
    * Identifies the largest detected face (based on bounding box area) as the primary target.
    * **Robot Control Logic:**
        * Calculates the horizontal deviation of the largest face's center from the image's center.
        * Generates a `geometry_msgs/msg/Twist` command with an `angular.z` (yaw) velocity proportional to this deviation. This command is published to the `/robotAction` topic. A "dead zone" prevents jittering when the face is near the center, and a maximum speed limits aggressive turns.
    * **Detection Info Publishing:**
        * Populates a `my_robot_interfaces/msg/DetectionInfo` message with the bounding box, center coordinates, and a `detected` flag. This message is published to the `/robotDetection/detection_info` topic.
    * **Visualization:**
        * Draws a green bounding box and a red circle at the center of the detected face on the image.
        * Displays the annotated image in a `cv2` window.

## Installation

Follow these steps to set up and run the project on your ROS 2 system. These instructions assume you are using **ROS 2 Humble** on Ubuntu, but the general steps apply to other distributions.

### Prerequisites

* **ROS 2 Humble (or compatible distribution):** Ensure your ROS 2 environment is sourced and ready.
* **Git:** For cloning the repository.
* **OpenCV:** Required for the face detection functionalities. You can typically install it via your package manager:
    ```bash
    sudo apt update
    sudo apt install libopencv-dev python3-opencv
    ```
* **Python 3:** Standard for ROS 2.
* **C++ Compiler:** Necessary for building the custom message package (`my_robot_interfaces`). Usually comes with `build-essential`.

### Setup Steps

1.  **Navigate to Your Preferred Workspace Location:**
    It's recommended to clone the repository into your home directory or a dedicated development folder. This repository structure is a full ROS 2 workspace.
    ```bash
    cd ~
    ```

2.  **Clone the Repository:**
    Clone this repository from GitHub.
    ```bash
    git clone [https://github.com/romy201/camera_bot_ros2_workspace.git](https://github.com/romy201/camera_bot_ros2_workspace.git)
    ```
    This command will create a new folder named `camera_bot_ros2_workspace` in your current directory, which will serve as your ROS 2 workspace root.

3.  **Navigate into the Workspace Root:**
    ```bash
    cd ~/camera_bot_ros2_workspace
    ```

4.  **Install ROS Dependencies:**
    Use `rosdep` to install any system dependencies required by the ROS packages in the workspace.
    ```bash
    rosdep install --from-paths src --ignore-src --rosdistro humble -y
    ```
    **Important:** Replace `humble` with your specific ROS 2 distribution (e.g., `iron`, `foxy`). If you haven't used `rosdep` before, you might need to initialize and update it first:
    ```bash
    sudo rosdep init
    rosdep update
    ```

5.  **Build the Workspace:**
    Now, compile all the packages (`my_robot_interfaces` and `camera_bot_pkg`) in your workspace.
    ```bash
    colcon build --symlink-install
    ```

6.  **Source the Environment:**
    After a successful build, you need to source the workspace's setup file in *each new terminal* you open to make the ROS 2 packages available.
    ```bash
    source install/setup.bash
    ```
    For convenience, you can add this line to your `~/.bashrc` file so it's automatically sourced when you open a new terminal:
    ```bash
    echo "source ~/camera_bot_ros2_workspace/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    (Adjust the path `~/camera_bot_ros2_workspace` if you cloned it to a different location.)

---

## Usage

Once installed and sourced, you will typically need **three separate terminal windows** to run all components effectively. Remember to `source install/setup.bash` in each new terminal if you haven't added it to your `.bashrc`.

1.  **Terminal 1: Run the Webcam Publisher**
    ```bash
    ros2 run camera_bot_pkg webcam_publisher
    ```
    This node will start publishing your webcam feed.

2.  **Terminal 2: Run the Human Detector Node**
    ```bash
    ros2 run camera_bot_pkg human_detector
    ```
    An OpenCV window should pop up displaying your webcam feed with detected faces (marked by green bounding boxes and red center circles). You will also see log messages in this terminal about detections and the generated angular actions.

3.  **Terminal 3: Monitor ROS 2 Topics (Optional, but Recommended)**
    Open a third terminal to `echo` the published topics and observe the data flow:
    * **Robot Control Commands:**
        ```bash
        ros2 topic echo /robotAction
        ```
    * **Detailed Detection Information:**
        ```bash
        ros2 topic echo /robotDetection/detection_info
        ```

---

## ROS 2 Topics

This project utilizes the following ROS 2 topics:

* `/camera/image_raw` (`sensor_msgs/msg/Image`): Input webcam video stream.
* `/robotAction` (`geometry_msgs/msg/Twist`): Output commands for robot's angular velocity (yaw).
* `/robotDetection/detection_info` (`my_robot_interfaces/msg/DetectionInfo`): Detailed information about detected faces.
