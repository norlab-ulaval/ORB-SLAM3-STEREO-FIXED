FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-22.04

# Install additional packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    cmake \
    build-essential \
    libopencv-dev \
    libeigen3-dev \
    libboost-serialization-dev \
    libssl-dev \
    libglew-dev \
    libepoxy-dev \
    pkg-config \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    python3-dev \
    python3-pip \
    git \
    wget \
    unzip

# Build Pangolin from source
WORKDIR /opt
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout eb51fea && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-Wno-error=type-limits -Wno-type-limits" && \
    make -j$(nproc) && \
    make install

WORKDIR /opt
RUN git clone https://github.com/norlab-ulaval/ORB-SLAM3-STEREO-FIXED.git -b offline orbslam3 && \
    cd orbslam3 && \
    git submodule update --init --recursive && \
    chmod +x build.sh && \
    ./build.sh

RUN cd /opt/orbslam3 && \
    wget https://github.com/norlab-ulaval/ORB_SLAM3_ROS2/raw/main/vocabulary/ORBvoc.txt.tar.gz && \
    tar -xvf ORBvoc.txt.tar.gz && \
    rm ORBvoc.txt.tar.gz

RUN mkdir /output
RUN mkdir /dataset

# install /opt/orbslam3/run.sh systsem-wide
RUN cp /opt/orbslam3/run.sh /usr/local/bin/run_orbslam3

# Clean up
RUN apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*
RUN ldconfig
