# ROS2 Divider Node Setup

This documentation explains how to create and run this ROS2 divider node using Docker on a Windows 11 machine.

The divider node subscribes to a topic, performs a division operation on two floating-point numbers, and publishes the result to another topic. 

Follow the steps below.

---

## Prerequisites

- **Windows 11** machine  
- **PowerShell (Admin mode)**  
- **Docker Desktop** installed  
- Basic knowledge of ROS2 and Docker

---

## Step-by-Step Instructions

### Pull the ROS2 Docker Image

Open PowerShell (in administrator mode) and run:

```bash
docker pull ros:humble
```

### Create and start the ROS2 Container

Run the following command to create and start a container named `ros2_container`:

```bash
docker run -it --name ros2_container ros:humble bash
```


### Source ROS2 environment

Source the ROS2 Humble setup file:

```bash
source /opt/ros/humble/setup.bash
```

### Create a workspace

```bash
mkdir -p ~/test_ws/src
```
### Create a package

```bash
ros2 pkg create test_pkg --build-type ament_cmake --dependencies rclcpp
```

### Copy the code

Copy the `divider_node.cpp` file from this Repository into the `~/test_ws/src/test_pkg/src` folder. 

Copy the `CMakeLists.txt` file from this Repository into the `~/test_ws/src/test_pkg/` folder.

### Build the package

Go to your workspace root `~/test_ws` and build the package using colcon:

```bash!
cd ~/test_ws
colcon build
```

### Source the workspace

After the build completes, source the workspace:

```bash!
source install/setup.bash
```

### Run the divider node

```bash!
ros2 run test_pkg divider_node
```

The node should now be running and waiting for messages on `/input_numbers`.

## Testing

In a new terminal (attach to the same container using `docker exec -it ros2_container bash`), you can publish test messages as follows:

```bash!
source /opt/ros/humble/setup.bash
source ~/test_ws/install/setup.bash
ros2 topic pub /input_numbers std_msgs/msg/Float64MultiArray '{ data: [10.0, 2.0] }'
```

To stop publishing the message, simply hit `Ctrl+C`

