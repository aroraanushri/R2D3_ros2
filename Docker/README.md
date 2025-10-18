# Docker Implementation for R2D3 ROS2

This directory contains Docker configuration files for building and running the R2D3 dual-arm composite lifting robot ROS2 environment.

## Overview

The Docker setup provides a containerized environment for the R2D3 ROS2 project, including:
- ROS2 Foxy desktop environment
- NVIDIA GPU support
- Pre-configured workspace with the R2D3 repository
- All necessary build tools and dependencies

## Prerequisites

- Docker installed on your system
- NVIDIA Docker runtime (for GPU support)
- Git (for cloning the repository)

## Quick Start

### Build the Docker Image

```bash
# From the project root directory
docker build -t r2d3_ros2:foxy ./Docker
```

### Run the Container

```bash
# Basic run (CPU only)
docker run -it --rm r2d3_ros2:foxy

# Run with GPU support
docker run -it --rm --gpus all r2d3_ros2:foxy

# Run with GPU and USB device access (for RealSense cameras)
docker run -it --rm --gpus all --device=/dev/bus/usb r2d3_ros2:foxy
```

## Detailed Usage

### Building the Image

The Dockerfile is designed to be lean and fast. It:
1. Uses `osrf/ros:foxy-desktop` as the base image
2. Installs essential build tools
3. Clones the R2D3 repository from GitHub
4. Sets up the ROS2 environment

```bash
# Build with custom repository URL (if needed)
docker build -t r2d3_ros2:foxy --build-arg REPO_URL=https://github.com/yourorg/R2D3_ros2.git ./Docker

# Build with specific branch
docker build -t r2d3_ros2:foxy --build-arg REPO_REF=develop ./Docker
```

### Running the Container

#### Basic Usage
```bash
docker run -it --rm r2d3_ros2:foxy
```

#### With GPU Support
```bash
docker run -it --rm --gpus all r2d3_ros2:foxy
```

#### With USB Device Access (for RealSense cameras)
```bash
docker run -it --rm --gpus all --device=/dev/bus/usb r2d3_ros2:foxy
```

#### With Volume Mounting (for development)
```bash
docker run -it --rm --gpus all \
  -v $(pwd):/workspace \
  r2d3_ros2:foxy
```

### Setting Up the Environment Inside Container

Once inside the container, you'll need to run the installation scripts:

```bash
# Source ROS2 environment
source /opt/ros/foxy/setup.bash

# Run ROS2 installation script
sudo bash ros2_rm_robot/dual_rm_install/scripts/ros2_install.sh

# Run MoveIt2 installation script
sudo bash ros2_rm_robot/dual_rm_install/scripts/moveit2_install.sh

# Install dependencies
rosdep update
rosdep install --from-paths . --ignore-src -r -y --rosdistro foxy

# Build the workspace
colcon build --packages-select rm_ros_interfaces
colcon build --packages-select realsense2_camera_msgs
source install/setup.bash
colcon build
```

## Docker Compose (Optional)

For easier management, you can create a `docker-compose.yml` file:

```yaml
version: '3.8'

services:
  r2d3_ros2:
    build:
      context: .
      dockerfile: Docker/Dockerfile
    image: r2d3_ros2:foxy
    container_name: r2d3_ros2
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=graphics
    volumes:
      - /dev/bus/usb:/dev/bus/usb
      - ./:/workspace
    devices:
      - /dev/bus/usb
    stdin_open: true
    tty: true
    command: bash
```

Then run with:
```bash
docker-compose up -d
docker-compose exec r2d3_ros2 bash
```

## Troubleshooting

### Common Issues

1. **Permission denied errors**
   ```bash
   # Run with proper user permissions
   docker run -it --rm --user root r2d3_ros2:foxy
   ```

2. **USB device not found**
   ```bash
   # Check USB devices
   lsusb
   
   # Run with proper USB access
   docker run -it --rm --privileged --device=/dev/bus/usb r2d3_ros2:foxy
   ```

3. **GPU not detected**
   ```bash
   # Verify NVIDIA Docker runtime
   docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
   ```

4. **Build failures**
   ```bash
   # Clean build cache
   docker build --no-cache -t r2d3_ros2:foxy ./Docker
   ```

### Environment Variables

The container sets these environment variables:
- `ROS_DISTRO=foxy`
- `ROS_WS=/root/ros2_ws`
- `NVIDIA_VISIBLE_DEVICES=all`
- `NVIDIA_DRIVER_CAPABILITIES=graphics`

## File Structure

```
Docker/
├── Dockerfile          # Main Docker configuration
├── .dockerignore       # Files to exclude from build context
└── README.md          # This file
```

## Development Workflow

1. **Make changes to your code**
2. **Rebuild the image** (if Dockerfile changes):
   ```bash
   docker build -t r2d3_ros2:foxy ./Docker
   ```
3. **Run the container**:
   ```bash
   docker run -it --rm --gpus all r2d3_ros2:foxy
   ```
4. **Test your changes** inside the container

## Contributing

When modifying Docker files:
1. Test the build process
2. Verify all functionality works
3. Update this README if needed
4. Ensure the image size remains reasonable

## Support

For issues related to Docker setup, please check:
1. This README for common solutions
2. The main project README for general setup
3. Create an issue in the project repository
