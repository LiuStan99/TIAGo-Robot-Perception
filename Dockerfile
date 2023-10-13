# Use an official Ubuntu 18.04 as a parent image
FROM palroboticssl/tiago_tutorials:melodic

# Set up non-interactive mode for package installation
ENV DEBIAN_FRONTEND=noninteractive

COPY Perception_packages /tiago_public_ws/src

# Source ROS Melodic setup.bash
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


