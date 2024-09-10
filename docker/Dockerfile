# Description: Dockerfile for the ROS noetic image with some additional packages
FROM osrf/ros:noetic-full

# Metadata
LABEL maintainer="Your Name <your.email@example.com>"
LABEL description="Dockerfile for the NavRover based on ROS Noetic"

# Update package list and install nano
RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

# Create a new user and set up the home directory
RUN useradd -ms /bin/bash rosuser

# Set the working directory
WORKDIR /home/rosuser/app

# Copy files from the local filesystem to the Docker image
COPY . /home/rosuser/app

# Change ownership of the copied files to the new user
RUN chown -R rosuser:rosuser /home/rosuser/app

# Switch to the new user
USER rosuser

# Command to run when the container starts
CMD ["bash"]
