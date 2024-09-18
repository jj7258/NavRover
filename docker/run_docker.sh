#!/bin/bash

# Allow local connections to the X server
xhost +local:root

# Set environment variables
export DISPLAY=:0
export XDG_RUNTIME_DIR=/tmp/runtime-root

# Check if NVIDIA runtime is available
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA runtime detected. Using GPU support."
    docker run -it --rm \
        --gpus all \  # Enable GPU support
        --network=host \
        --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=$DISPLAY \
        navrover
else
    echo "No NVIDIA runtime detected. Using Intel integrated graphics."
    docker run -it --rm \
        --network=host \
        --ipc=host \
        --device=/dev/dri \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=$DISPLAY \
        navrover
fi