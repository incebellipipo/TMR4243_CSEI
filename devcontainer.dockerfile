# Use the official Ubuntu 24.04 (Noble) base image
FROM incebellipipo/devcontainer:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
# Source ROS2 setup script
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Create workspace
RUN mkdir -p /home/developer/ros_ws/src

# Clone the repositories
WORKDIR /home/developer/ros_ws/src

# RUN git clone https://github.com/NTNU-MCS/TMR4243_LAB.git && \
#     cd TMR4243_LAB && \
#     git submodule update --init --recursive

RUN git clone https://github.com/NTNU-MCS/cybership_software_suite.git && \
    cd cybership_software_suite && \
    git submodule update --init --recursive

# Install ROS dependencies
WORKDIR /home/developer/ros_ws
RUN \
    sudo apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN sudo rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Source the workspace setup script
RUN echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]