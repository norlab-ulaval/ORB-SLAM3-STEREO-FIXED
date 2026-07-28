#!/bin/bash
# Run ORB-SLAM3 on a stereo-inertial dataset

# DATASET_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-06-26-10-31/sensors"
# OUTPUT_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-06-26-10-31/process_june_realtime"
# TRAJECTORY_NAME="red_2025-06-26-10-31"

# DATASET_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/sensors"
# OUTPUT_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/process_june"
# TRAJECTORY_NAME="orange_2025-06-26-10-48"

ORB_DATASET_PATH=${ORB_DATASET_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-06-26-10-31/sensors"}
ORB_OUTPUT_PATH=${ORB_OUTPUT_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-06-26-10-31/sensors/process_june_realtime"}
ORB_TRAJECTORY_NAME=${ORB_TRAJECTORY_NAME:-"red_2025-06-26-10-31"}
ORB_LOAD_ATLAS=${ORB_LOAD_ATLAS:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-06-26-10-31/atlas"}

if [ -z "$ORB_DATASET_PATH" ]; then
    echo "Error: ORB_DATASET_PATH is not set."
    exit 1
fi

if [ -z "$ORB_OUTPUT_PATH" ]; then
    echo "Error: ORB_OUTPUT_PATH is not set."
    exit 1
fi

if [ -z "$ORB_TRAJECTORY_NAME" ]; then
    echo "Error: ORB_TRAJECTORY_NAME is not set."
    exit 1
fi

echo "Trajectory name is: ${ORB_TRAJECTORY_NAME}"

# Update the YAML configuration
/workspaces/ORB-SLAM3-STEREO-FIXED/configure.py --load-atlas "$ORB_LOAD_ATLAS"

# Run ORB-SLAM3
mkdir -p $ORB_OUTPUT_PATH
cd $ORB_OUTPUT_PATH
/workspaces/ORB-SLAM3-STEREO-FIXED/Examples/Stereo-Inertial/stereo_inertial_fomo /workspaces/ORB-SLAM3-STEREO-FIXED/Vocabulary/ORBvoc.txt /workspaces/ORB-SLAM3-STEREO-FIXED/fomo_scripted.yaml $ORB_DATASET_PATH $ORB_TRAJECTORY_NAME # --realtime
