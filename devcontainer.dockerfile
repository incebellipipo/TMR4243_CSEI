# Use the official Ubuntu 24.04 (Noble) base image
FROM incebellipipo/devcontainer:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN sudo apt-get update && sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-numpy \
    python3-scipy \
    git \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup script
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


# Create workspace
RUN mkdir -p /home/developer/ros_ws/src

# Clone the repositories
WORKDIR /home/developer/ros_ws/src
RUN git clone https://github.com/NTNU-MCS/TMR4243_LAB.git
RUN git clone https://github.com/NTNU-MCS/cybership_software_suite.git

# Install ROS dependencies
WORKDIR /home/developer/ros_ws
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source the workspace setup script
RUN echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]