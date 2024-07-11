# Use the official ROS Noetic robot image as the base image
FROM ros:noetic-robot

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-opencv \
    python3-yaml \
    python3-dev \
    libi2c-dev \
    i2c-tools \
    libusb-1.0-0-dev \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# Install Jetson GPIO
RUN pip3 install Jetson.GPIO

# Set up the catkin workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set up entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Set the working directory
WORKDIR /root/catkin_ws

# Run bash when the container launches
CMD ["bash"]
