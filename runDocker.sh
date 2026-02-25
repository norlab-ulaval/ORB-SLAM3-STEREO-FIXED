#!/bin/bash

input_path_host=/home/mbo/bigfoot-FoMo/ijrr
output_path_host=/home/mbo/output/orbslam3-offline


docker run -it --rm \
    -v ${input_path_host}/2025-10-14/red_2025-10-14-11-41:/dataset \
    -v ${output_path_host}/2025-10-14/red_2025-10-14-11-41:/output \
    -v $HOME/orbslam3:/opt/orbslam3 \
    -e DATASET_PATH=/dataset \
    -e OUTPUT_PATH=/output \
    -e TRAJECTORY_NAME=red_2025-10-14-11-41_red_2025-10-14-11-41 \
    orbslam3offline:latest bash
