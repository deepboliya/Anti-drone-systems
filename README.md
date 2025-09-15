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
- **PX4 Autopilot dependencies**: See [PX4 Dev Guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- **Gazebo Harmonic (Recommended)**: Required for SITL simulation. [Install here](https://gazebosim.org/docs/harmonic/install_ubuntu). _Do not install both Gazebo Harmonic and Gazebo Classic on the same system, as this can cause conflicts._
- **ROS 2** (for integration): Make sure you have a working ROS 2 installation
- **Micro XRCE-DDS Agent** (for ROS2-uORB bridge):
  ```bash
  sudo snap install micro-xrce-dds-agent
  ```
- **Python dependencies**:
  ```bash
  pip install -r requirements.txt
  ```

## Installation

### PX4 Autopilot
Run the PX4 setup script:
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

### Build & Launch the Simulator
To build and launch the Gazebo-based anti-drone simulation:
```bash
make px4_sitl_antidrone gz_x500
```

## ROS 2 Integration

1. **Start the Micro XRCE-DDS Agent** (in a separate terminal):
   ```bash
   micro-xrce-dds-agent udp4 -p 8888
   ```
   [Micro XRCE-DDS documentation](https://micro-xrce-dds.docs.eprosima.com/en/latest/introduction.html)

2. **Build the ROS 2 workspace**:
   ```bash
   cd ros2_uorb_inerface
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Run a ROS 2 node** (example):
   ```bash
   ros2 run px4_ros_com sensor_combined_listener
   ```
   > **Note:** Make sure you have sourced the ROS 2 overlay:
   > ```bash
   > source /opt/ros/$ROS_DISTRO/setup.bash
   > ```

## Additional Resources
- [PX4 Documentation](https://docs.px4.io/)
- [Gazebo Documentation](https://gazebosim.org/)
- [ROS 2 Documentation](https://docs.ros.org/)

---

For questions, issues, or contributions, please open an issue or pull request on GitHub. Happy flying!




