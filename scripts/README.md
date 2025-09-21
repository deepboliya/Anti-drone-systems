# Scripts

Python utilities for drone communication and control using MAVSDK.

## Overview

This directory contains MAVSDK-based scripts for interfacing with PX4-powered drones. MAVSDK is easier to understand and use compared to ROS2 based communication

📖 **Reference**: [px4 mavsdk](https://docs.px4.io/main/en/robotics/mavsdk.html)

## Prerequisites
- Use requirements.txt

## Scripts

| Script | Description |
|--------|-------------|
| `mimic_ground_station.py` | Establishes MAVLink connection without QGroundControl, enables arming w/o QGC|
| `print_coordinates.py` | Logs drone position data for analysis |
| `square.py` | Autonomous square flight pattern with takeoff/landing |

## Usage

```bash
# Basic connection + helps arming w/o QGC
python mimic_ground_station.py

# Position monitor. (read known issues)
python print_coordinates.py

# Autonomous square flight
python square.py
```

## Known Issues

**Coordinate Updates**: `print_coordinates.py` only reads initial position (requires investigation)
