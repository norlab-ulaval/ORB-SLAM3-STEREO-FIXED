#!/bin/bash

docker run -it --rm \
    -v $HOME/orbslam3:/opt/orbslam3 \
    orbslam3offline:latest bash -c "cd /opt/orbslam3 && ./build.sh"
