FROM ubuntu:18.04

LABEL maintainer = "Michele Adduci <adduci.michele@gmail.com>" \
      description = "Dockerfile for embedded C++/Point Cloud Library development environment"

ARG PCL_VERSION_ARG=pcl-1.9.1
ENV PCL_VERSION=${PCL_VERSION_ARG}

RUN echo "Installing basic dependencies" && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      build-essential \
      cmake \
      cmake-curses-gui \
      pkg-config \
      make \
      gdb \
      git \
      libboost-all-dev \
      libvtk7-dev \
      libeigen3-dev \
      libflann-dev && \
    apt-get autoclean -y && \
    apt-get autoremove -y --purge && \
    rm -rf /tmp/* && \
    rm -rf /var/log/apt/*

WORKDIR /code

RUN echo "Downloading PointCloudLibrary from GitHub" && \
    git clone https://github.com/PointCloudLibrary/pcl.git /code/pcl

RUN echo "Preparing to build ${PCL_VERSION}" && \
    cd /code/pcl && \
    git checkout tags/${PCL_VERSION} && \
    mkdir build && \
    cd build/ && \
    cmake -D WITH_OPENNI=OFF -D WITH_OPENNI2=OFF -D WITH_QT=OFF .. && \
    make -j4 && \
    make install && \
    echo "Installation of PCL completed"