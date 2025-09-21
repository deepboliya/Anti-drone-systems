# Anti-Drone Project

This repository provides a simulation and development environment for the anti-drone research, using PX4 Autopilot, ROS 2, and Gazebo

## Getting Started

### 1. Clone the Repository
Clone with all submodules:
```bash
git clone https://github.com/deepboliya/Anti-drone-systems.git --recurse-submodules
cd PX4-Autopilot-ADS
git checkout main
```

### 2. Prerequisites
<!-- - **PX4 Autopilot dependencies**: See [PX4 Dev Guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) -->
- **Gazebo Harmonic (Recommended)**: Required for SITL simulation. [Install here](https://gazebosim.org/docs/harmonic/install_ubuntu). _Do not install both Gazebo Harmonic and Gazebo Classic on the same system, as this can cause conflicts._
- **ROS 2** (for integration): Make sure you have a working ROS 2 installation
- **Micro XRCE-DDS Agent** (for ROS2-uORB bridge):
  ```bash
  sudo snap install micro-xrce-dds-agent
  ```
  > **Note:** Snap doesn't work over _IITB-Wireless_, you will need personal hotspot for this to work. [Standalone installation](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html) is possible without using Snap
- **Python dependencies**:
  ```bash
  pip install -r requirements.txt
  ```

## Installation

### PX4 Autopilot
Run the PX4 setup script:
```bash
bash ./PX4-Autopilot-ADS/Tools/setup/ubuntu.sh
```

### Build & Launch the Simulator
To build and launch the Gazebo-based anti-drone simulation:
```bash
cd PX4-Autopilot-ADS
make px4_sitl_antidrone gz_x500
```

## ROS 2 Integration

1. **Start the Micro XRCE-DDS Agent** (in a separate terminal):
   ```bash
   MicroXRCEAgent udp4 -p 8888 # For manual installation
   # or
   micro-xrce-dds-agent udp4 -p 8888 # For snap installation
   ```
   [Micro XRCE-DDS documentation](https://micro-xrce-dds.docs.eprosima.com/en/latest/introduction.html)

2. **Build the ROS 2 workspace**:
   ```bash
   cd ros2_uorb_inerface
   colcon build --symlink-install
   source install/setup.bash
   ```
3. **Listen to topics on terminal**
   ```bash
   ros2 topic echo 
   ```
3. **Run a ROS 2 node** (example):
   ```bash
   ros2 run px4_ros_com sensor_combined_listener
   ```
   > **Note:** _px4_msgs_ package takes time to build(~4min)

   > **Note:** Make sure you have sourced the ROS 2 overlay:
   > ```bash
   > source /opt/ros/$ROS_DISTRO/setup.bash
   > ```
## Simulation Configuration

### IMU Frequency Modification
To change IMU frequency, modify these files:
- `src/modules/sensors/vehicle_imu/imu_parameters.c`
- `Tools/simulation/gz/models/x500_base/model.sdf`
- `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator`

> **Note:** IMU frequency must be a multiple of the step size in `Tools/simulation/gz/worlds/default.sdf`
## Additional Resources
- [PX4 Documentation](https://docs.px4.io/)
- [Gazebo Documentation](https://gazebosim.org/)
- [ROS 2 Documentation](https://docs.ros.org/)

---


