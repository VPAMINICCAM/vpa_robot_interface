FROM ros:noetic-robot-focal

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Berlin

# Update and install Python setuptools and other dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-setuptools \
    python3-pip \
    sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages with pip
RUN pip3 install traitlets Jetson.GPIO

# Create a non-root user
RUN useradd -m -s /bin/bash vpaadmin \
    && echo "vpaadmin:vpaduck" | chpasswd \
    && echo "vpaadmin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER vpaadmin
WORKDIR /home/vpaadmin

# Create and initialize the catkin workspace
RUN mkdir -p ~/catkin_ws/src \
    && /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace ~/catkin_ws/src'

# Copy the ROS project and the jetracer project into the container
COPY ./vpa_robot_interface /home/vpaadmin/catkin_ws/src/vpa_robot_interface
COPY ./jetracer /home/vpaadmin/jetracer

# Setup ROS environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /home/vpaadmin/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "export ROS_MASTER_URI=http://192.168.1.4:11311" >> ~/.bashrc \
    && echo "export ROS_IP=\$(hostname -I | awk '{print \$1}')" >> ~/.bashrc \
    && echo "export ROS_HOSTNAME=\$ROS_IP" >> ~/.bashrc

# Build the ROS project
WORKDIR /home/vpaadmin/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Install the jetracer Python package
WORKDIR /home/vpaadmin/jetracer
RUN sudo python3 setup.py install

# Set the default working directory to the home directory of 'vpaadmin'
WORKDIR /home/vpaadmin

CMD ["bash"]
