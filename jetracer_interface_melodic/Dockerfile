# Use an NVIDIA L4T base image compatible with Jetson Nano
FROM nvcr.io/nvidia/l4t-base:r32.6.1

# Avoid interactive dialogue with apt-get and set timezone
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Berlin

# Update and install dependencies, including ROS Melodic and utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    gnupg2 \
    sudo \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y \
    ros-melodic-ros-base \
    ros-melodic-image-transport \
    ros-melodic-image-transport-plugins \
    ros-melodic-compressed-image-transport \
    ros-melodic-gscam \
    python-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
RUN useradd -m -s /bin/bash vpaadmin \
    && echo "vpaadmin:vpaadmin" | chpasswd \
    && echo "vpaadmin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER vpaadmin
WORKDIR /home/vpaadmin

# Initialize rosdep
RUN sudo rosdep init \
    && rosdep update

# Setup ROS environment and automatically source the ROS setup script on container startup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && echo "source /home/vpaadmin/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "export ROS_MASTER_URI=http://192.168.1.4:11311" >> ~/.bashrc \
    && echo "export ROS_IP=\$(hostname -I | awk '{print \$1}')" >> ~/.bashrc \
    && echo "export ROS_HOSTNAME=\$ROS_IP" >> ~/.bashrc

# Create and initialize the catkin workspace
RUN mkdir -p ~/catkin_ws/src \
    && /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace ~/catkin_ws/src'

# Copy the ROS project into the container
COPY ./vpa_robot_interface /home/vpaadmin/catkin_ws/src/vpa_robot_interface

# Build the ROS project
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'

# Make the container start in the catkin workspace
WORKDIR /home/vpaadmin

CMD ["bash"]
