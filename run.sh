#!/bin/bash
# Run ORB-SLAM3 on a stereo-inertial dataset

if [ -z "$DATASET_PATH" ]; then
    echo "Error: DATASET_PATH is not set."
    exit 1
fi

if [ -z "$OUTPUT_PATH" ]; then
    echo "Error: OUTPUT_PATH is not set."
    exit 1
fi


# Run ORB-SLAM3
cd /output
/opt/orbslam3/Examples/Stereo-Inertial/stereo_inertial_fomo /opt/orbslam3/ORBvoc.txt /opt/orbslam3/fomo.yaml /dataset
