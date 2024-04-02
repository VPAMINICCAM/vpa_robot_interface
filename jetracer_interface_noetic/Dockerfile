FROM ros:noetic-robot-focal

ENV DEBIAN_FRONTEND=noninteractive

# Create a non-root user
RUN useradd -m -s /bin/bash vpaadmin \
    && echo "vpaadmin:vpaduck" | chpasswd \
    && echo "vpaadmin ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers