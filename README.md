# KITTI to ROS 2 Bag Converter

This repository converts KITTI tracking dataset sequences into ROS 2 bag files, synchronizing:
- **Images** (`/camera/image_raw`)
- **Point Clouds** (`/lidar/points`)
- **2D Detections** (`/detection_2d/car`, `/detection_2d/pedestrian`)
- **3D Detections** (`/detection_3d/car`, `/detection_3d/pedestrian`)
- **Odometry** (`/sensor/odometry`)
- **Calibration Data** (`/camera/calibration`)

---

## Requirements

- ROS 2 (tested with Humble)
---

## Installation and Setup

1. **Install required ROS 2 packages:**
    ```bash
    sudo apt update
    sudo apt install ros-humble-rclpy ros-humble-rosbag2-py ros-humble-cv-bridge python3-opencv python3-numpy
    ```

2. **Build the custom message:**
    Make sure you have the `my_msgs` package with the custom message `Float32MultiArrayStamped` (Only if you are using the precomputed detections).

    ```bash
    cd <your_ros2_workspace>/src
    colcon build --packages-select my_msgs
    ```

## Clone the repository

This repository should be inside your ROS 2 workspace. Clone it using:
```bash
cd <your_ros2_workspace>/src
git clone https://github.com/Prabuddhi-05/rosbag_convert.git
colcon build --packages-select rosbag_convert
source install/setup.bash

```

Clone the `my_msgs` package ([LINK HERE](https://github.com/Prabuddhi-05/my_msgs.git)) (only if using precomputed detections):
```bash
cd <your_ros2_workspace>/src
git clone [https://github.com/Prabuddhi-05/my_msgs.git](https://github.com/Prabuddhi-05/my_msgs.git)
colcon build --packages-select my_msgs

## Usage

To run the code, use the following command:
```bash
ros2 run rosbag_convert kitti_to_ros
```

You can adjust the following in the code as needed:
- **`base_path`**: Path to the KITTI dataset
- **`output_bag_path`**: Path where ROS 2 bag files will be saved
- **`seq_id`**: Number of sequences to process (default is 29)
- **`Paths to detections`**: Adjust the paths for 2D and 3D detection files in the code (Only if you are using the precomputed detections)

---


# KITTI to ROS 2 Bag Converter

This repository converts KITTI tracking dataset sequences into ROS 2 bag files, synchronizing:

- **Images** (`/camera/image_raw`)
- **Point Clouds** (`/lidar/points`)
- **2D Detections** (`/detection_2d/car`, `/detection_2d/pedestrian`)
- **3D Detections** (`/detection_3d/car`, `/detection_3d/pedestrian`)
- **Odometry** (`/sensor/odometry`)
- **Calibration Data** (`/camera/calibration`)

---

## Requirements

- ROS 2 (tested with Humble)

---

## Installation and Setup

1. **Install Required ROS 2 Packages:**

   ```bash
   sudo apt update
   sudo apt install ros-humble-rclpy ros-humble-rosbag2-py ros-humble-cv-bridge python3-opencv python3-numpy
   ```

2. **Build the Custom Message:** Make sure you have the `my_msgs` package with the custom message `Float32MultiArrayStamped`.

   ```bash
   cd <your_ros2_workspace>
   colcon build --packages-select my_msgs
   source install/setup.bash
   ```

## Clone the Repository

Clone the `my_msgs` package ([LINK HERE](https://github.com/Prabuddhi-05/my_msgs.git)) (only if using precomputed detections):
```bash
cd <your_ros2_workspace>/src
git clone [https://github.com/Prabuddhi-05/my_msgs.git](https://github.com/Prabuddhi-05/my_msgs.git)
colcon build --packages-select my_msgs
```

This repository should be inside your ROS 2 workspace. Clone it using:

This repository should be inside your ROS 2 workspace. Clone it using:

```bash
cd <your_ros2_workspace>/src
git clone https://github.com/Prabuddhi-05/rosbag_convert.git
cd rosbag_convert
```

## Usage

To run the code, use the following command:

```bash
ros2 run rosbag_convert kitti_to_ros
```

You can adjust the following in the code as needed:

- ``: Path to the KITTI dataset
- ``: Path where ROS 2 bag files will be saved
- ``: Number of sequences to process (default is 29)
- **Paths to detections**: Adjust the paths for 2D and 3D detection files in the code

---



