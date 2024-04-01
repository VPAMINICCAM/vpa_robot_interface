# Use the ros:melodic-robot base image
FROM ros:melodic-robot

# Set environment variables to avoid user interaction during apt-get install
ENV DEBIAN_FRONTEND noninteractive

# Create a new user 'vpaadmin' with password 'vpaduck'
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    ros-melodic-gscam && \
    useradd -m -s /bin/bash vpaadmin && \
    echo "vpaadmin:vpaduck" | chpasswd && \
    echo "vpaadmin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create the catkin workspace directories
RUN mkdir -p /home/vpaadmin/catkin_ws/src

# Copy your local project directory into the Docker image
# Replace `./vpa_robot_interface` with the path to your local project directory
COPY ./vpa_robot_interface /home/vpaadmin/catkin_ws/src/vpa_robot_interface

# Change ownership of the catkin_ws to vpaadmin
RUN chown -R vpaadmin:vpaadmin /home/vpaadmin/catkin_ws

# Switch to the newly created user
USER vpaadmin

# Change to the catkin_ws directory
WORKDIR /home/vpaadmin/catkin_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Add ROS environment sourcing to .bashrc for new interactive bash sessions
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/vpaadmin/.bashrc && \
    echo "export ROS_MASTER_URI=http://192.168.1.4:11311" >> /home/vpaadmin/.bashrc && \
    echo "export ROS_IP=\$(hostname -I | awk '{print \$1}')" >> /home/vpaadmin/.bashrc && \
    echo "export ROS_HOSTNAME=\$ROS_IP" >> /home/vpaadmin/.bashrc && \
    echo "source /home/vpaadmin/catkin_ws/devel/setup.bash" >> /home/vpaadmin/.bashrc

# Switch to root to configure the entrypoint
USER root

# Set the entrypoint and default command
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
USER vpaadmin
CMD ["/bin/bash"]
