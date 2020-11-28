FROM ros:noetic
LABEL maintainer="matthew.scott.hannay@gmail.com"

# Setup apt to be happy with no console input
ARG DEBIAN_FRONTEND=noninteractive

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing && apt-get -y install \
    apt-utils \
    git \
    software-properties-common \
    ssh \
    python3-pip \
    libeigen3-dev \
    && apt-get clean

# Initialize catkin workspace
RUN mkdir -p /catkin_ws
WORKDIR /catkin_ws
RUN mkdir -p src

COPY . /catkin_ws/src/igvc-software

# Install all ROS dependencies that can automatically be installed
WORKDIR /catkin_ws/src/igvc-software
RUN /bin/bash -c /catkin_ws/src/igvc-software/install_dependencies.sh
