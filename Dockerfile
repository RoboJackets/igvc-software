FROM ros:melodic
MAINTAINER Matthew Barulic matthew.barulic@gmail.com

# Setup apt to be happy with no console input
ENV DEBIAN_FRONTEND noninteractive

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing \
    && apt-get -y install apt-utils git software-properties-common ssh python-pip libeigen3-dev \
    && apt-get clean

# Install kindr from source
RUN git clone https://github.com/ANYbotics/kindr \
    && cd kindr \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install \
    && cd ../../ \
    && rm -rf kindr

# Initialize catkin workspace
RUN mkdir -p ~/catkin_ws
WORKDIR ~/catkin_ws
RUN mkdir -p src

COPY . ./src/igvc-software

# Install all ROS dependencies that can automatically be installed
RUN /bin/bash -c "rosdep install -iy --from-paths ./src --skip-keys='python-pytorch-pip' && pip install --no-cache-dir torch torchvision"
