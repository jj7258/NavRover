# NavRover Docker

NavRover is a ROS-based project designed for autonomous navigation. This repository provides a Docker setup to simplify the installation and execution of the NavRover application.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Building the Docker Image](#building-the-docker-image)
- [Running the Docker Container](#running-the-docker-container)
- [Entry Point](#entry-point)
- [License](#license)

## Prerequisites

- Docker installed on your machine.
- NVIDIA GPU (optional) for GPU support.

## Building the Docker Image

To build the Docker image, navigate to the `docker` directory and run the following command:


    ./build_docker.sh


This script will build the Docker image with the name `navrover`. Ensure that the build completes successfully.

## Running the Docker Container

To run the Docker container, execute the following command:


    ./run_docker.sh


This script will check for NVIDIA runtime availability and run the container accordingly, enabling GPU support if available.

### Environment Variables

- `DISPLAY`: Set to `:0` for X11 forwarding.
- `XDG_RUNTIME_DIR`: Set to `/tmp/runtime-root` for GUI applications.

## Entry Point

The entry point for the Docker container is defined in `entrypoint.sh`, which sets up the ROS environment and executes the command passed to the container.
