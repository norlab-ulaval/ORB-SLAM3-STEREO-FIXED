#!/bin/bash
# Run ORB-SLAM3 on a stereo-inertial dataset

# DATASET_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-06-26-10-31/sensors"
# OUTPUT_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-06-26-10-31/process_august"
# TRAJECTORY_NAME="red_2025-06-26-10-31"

# DATASET_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/sensors"
# OUTPUT_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/process_june"
# TRAJECTORY_NAME="orange_2025-06-26-10-48"


DATASET_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/yellow_2025-10-14-14-48/"
OUTPUT_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/yellow_2025-10-14-14-48/process_october"
TRAJECTORY_NAME="yellow_2025-10-14-14-48"


if [ -z "$DATASET_PATH" ]; then
    echo "Error: DATASET_PATH is not set."
    exit 1
fi

if [ -z "$OUTPUT_PATH" ]; then
    echo "Error: OUTPUT_PATH is not set."
    exit 1
fi

if [ -z "$TRAJECTORY_NAME" ]; then
    echo "Error: TRAJECTORY_NAME is not set."
    exit 1
fi


echo "Trajectory name is: ${TRAJECTORY_NAME}"


# Run ORB-SLAM3
mkdir -p $OUTPUT_PATH
cd $OUTPUT_PATH
/workspaces/ORB-SLAM3-STEREO-FIXED/Examples/Stereo-Inertial/stereo_inertial_fomo /workspaces/ORB-SLAM3-STEREO-FIXED/Vocabulary/ORBvoc.txt /workspaces/ORB-SLAM3-STEREO-FIXED/fomo.yaml $DATASET_PATH $TRAJECTORY_NAME
