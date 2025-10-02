# Spot Multi-Robot Control

A ROS2 package for controlling multiple Boston Dynamics Spot robots with fiducial-based localization.

Changelogs:
- 09-29-2025: initial documentation by claude and modified by wyc for correctness.

## Setup Instructions

### 1. Create ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Clone and Install Spot ROS2

```bash
cd ~/ros2_ws/src
git clone https://github.com/bdaiinstitute/spot_ros2.git
cd spot_ros2
./install_spot_ros2.sh
```

### 3. Clone and Setup ROS Sharp

```bash
cd ~
git clone https://github.com/siemens/ros-sharp.git
# Copy over file_server2 (adjust path as needed)
cp -r "ros-sharp/ROS Packages/ROS2/file_server2" ~/ros2_ws/src/
```

### 4. Clone This Repository

```bash
cd ~/ros2_ws/src
git clone github.com/h2r/spot_ros2_multi spot_multi
```

### 5. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
```

### 6. Create Spot Configuration Directory

```bash
mkdir -p ~/spot_configs
```

Copy the example configuration files (see Configuration section below) to `~/spot_configs/`.

## Configuration

Create the following configuration files in `~/spot_configs/`:

### spot_gouger.yaml
```yaml
/**:
  ros__parameters:
    # Spot Login Information
    username: "user"
    password: "your_spot_password"
    hostname: "192.168.1.100"  # Replace with your Spot's IP

    # Status Updates from Spot
    metrics_rate: 0.04
    lease_rate: 1.0
    async_tasks_rate: 10.0
    robot_state_rate: 50.0
    image_rate: 10.0

    # Boolean parameters
    auto_claim: True
    auto_power_on: False
    auto_stand: False

    # Estop Parameters
    estop_timeout: 9.0
    start_estop: False

    preferred_odom_frame: "vision"
    tf_root: "body"

    # Robot identification
    spot_name: "gouger"

    cmd_duration: 0.25
    arm_cmd_duration: 1.0
    rgb_cameras: True
    initialize_spot_cam: False

    use_velodyne: True
    velodyne_rate: 10.0

    # Virtual camera parameters for image stitching
    virtual_camera_intrinsics: [385.0, 0.0, 315.0, 0.0, 385.0, 844.0, 0.0, 0.0, 1.0]
    virtual_camera_projection_plane: [-0.15916, 0.0, 0.987253]
    virtual_camera_plane_distance: 0.5
    stitched_image_row_padding: 1182

    gripperless: False
    publish_graph_nav_pose: True
```

## Running the System

### Terminal 1: Source and Launch Spot Gouger
```bash
cd ~/ros2_ws
source install/setup.zsh
ros2 launch spot_driver spot_driver.launch.py config_file:=$HOME/spot_configs/spot_gouger.yaml
```

### Terminal 2: Launch Spot Tusker
```bash
cd ~/ros2_ws
source install/setup.zsh
ros2 launch spot_driver spot_driver.launch.py config_file:=$HOME/spot_configs/spot_tusker.yaml
```

### Terminal 3: Launch ROS Sharp Communication
```bash
cd ~/ros2_ws
source install/setup.zsh
ros2 launch file_server2 ros_sharp_communication.launch.py
```

### Terminal 4: Run GraphNav Localization, ICP of the Point clouds, and sync drive.
```bash
cd ~/ros2_ws
source install/setup.zsh
ros2 launch spot_multi spot_multi.launch.py
```
- GraphNav base localization is used. Lidar ICP registration will be applied on top of GraphNav
  in `/spot_tf_compute` only, for unity consumption.
- Currently, the ICP uses hard-coded topic names. You may edit the cpp source code to change
  the topics. In the future, it will be re-written to be more general.
- Currently, the `/multi_spot/cmd_vel` is velocity based, which is not stable enough.


## Notes

- Make sure both Spot robots are powered on and connected to the network
- Update the IP addresses in the configuration files to match your robots
- Ensure you have the correct username and password for your Spot robots
- The fiducial localization script should be run after all other components are started
