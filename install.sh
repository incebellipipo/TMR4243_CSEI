#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Check if ROS is installed using apt
if ! dpkg -l | grep ros-jazzy; then
    echo "ROS Jazzy is not installed. Please install ROS Jazzy first."
    exit 1
fi

# Confirm the installation
echo "This script will install the necessary dependencies for the CyberShip software suite."
# Summary of the installation
echo "The following steps will be performed:"
echo "1. Update and upgrade the system"
echo "2. Install necessary dependencies"
echo "3. Create a workspace"
echo "4. Clone the repositories"
echo "5. Get all the submodules for cybership_software_suite"
echo "6. Build a virtual environment"
echo "7. Install python dependencies to virtual environment"
echo "8. Install ROS dependencies"
echo "9. Build the workspace"
echo "10. Source the workspace setup script"
# Ask for confirmation
read -p "Do you want to continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Install necessary dependencies
sudo apt install -y python3-pip python3-numpy python3-scipy git python3-venv

# Create a workspace
mkdir -p ~/ros_ws/src

# Clone the repositories
cd ~/ros_ws/src
git clone https://github.com/NTNU-MCS/TMR4243_LAB.git
git clone https://github.com/NTNU-MCS/cybership_software_suite.git

# Get all the submodules for cybership_software_suite
cd ~/ros_ws/src/cybership_software_suite
git submodule update --init --recursive

# Build a virtual environment
cd ~/ros_ws
python3 -m venv venv --system-site-packages --symlinks
source venv/bin/activate
touch venv/COLCON_IGNORE

# Install python dependencies to virtual environment
find src/cybership_software_suite -name "requirements*txt" -exec pip install -r {} \;

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source the workspace setup script
echo "source ~/ros_ws/venv/bin/activate" >> ~/.bashrc
echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Installation completed successfully!"