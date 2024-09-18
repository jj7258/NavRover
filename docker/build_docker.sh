#!/bin/bash

# Set the image name
IMAGE_NAME="navrover"

# Print start message
echo "Starting to build Docker image: ${IMAGE_NAME}"

# Build the Docker image
docker build -t ${IMAGE_NAME} .

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "Docker image built successfully: ${IMAGE_NAME}"
else
    echo "Error: Docker image build failed"
    exit 1
fi

# Optionally, you can add a command to list the images to confirm the build
docker images | grep ${IMAGE_NAME}

echo "Build process completed."

