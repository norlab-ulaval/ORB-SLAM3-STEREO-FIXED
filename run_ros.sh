#!/bin/bash
# Run ORB-SLAM3 on a stereo-inertial dataset

# ORB_TRAJECTORY_NAME=${ORB_TRAJECTORY_NAME:-"red_2025-06-26-10-31"}
# ORB_OUTPUT_PATH=${ORB_OUTPUT_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-08-20-10-42/process_june_rosbag"}
# ORB_LOAD_ATLAS=${ORB_LOAD_ATLAS:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-08-20-10-42/atlas"}
# ORB_OUTPUT_PATH=${ORB_OUTPUT_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/process_june_rosbag"}
# ORB_LOAD_ATLAS=${ORB_LOAD_ATLAS:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/atlas"}

# ORB_DATASET_PATH=${ORB_DATASET_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-01-30-09-07/"}
ORB_DATASET_PATH=${ORB_DATASET_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-08-20-13-17/"}
# ORB_DATASET_PATH=${ORB_DATASET_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/"}
ORB_LOAD_ATLAS=${ORB_LOAD_ATLAS:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/atlas"}

ORB_TRAJECTORY_NAME=${ORB_TRAJECTORY_NAME:-"$(basename "${ORB_DATASET_PATH%/}")"}
ORB_OUTPUT_PATH=${ORB_OUTPUT_PATH:-"$(dirname "${ORB_LOAD_ATLAS%/}")/process_jan-loc"}

export ORB_REALTIME=${ORB_REALTIME:-true}

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

# Real-time mode. Exported so configure.py picks the matching lightweight
# feature settings: this single variable drives both the YAML params and the
# frame-dropping flag below, so the two cannot disagree.
REALTIME_FLAG=""
case "${ORB_REALTIME,,}" in
    1|true|yes|on) REALTIME_FLAG="--realtime" ;;
esac

if [ -n "$REALTIME_FLAG" ]; then
    echo "Realtime mode: ENABLED (frame dropping + lightweight features)"
else
    echo "Realtime mode: disabled (process every frame at full quality)"
fi

# Update the YAML configuration
/workspaces/ORB-SLAM3-STEREO-FIXED/configure.py --load-atlas "$ORB_LOAD_ATLAS"
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Run ORB-SLAM3
mkdir -p $ORB_OUTPUT_PATH
cd $ORB_OUTPUT_PATH
/workspaces/ORB-SLAM3-STEREO-FIXED/Examples/Stereo-Inertial/stereo_inertial_fomo_ros \
     /workspaces/ORB-SLAM3-STEREO-FIXED/Vocabulary/ORBvoc.txt \
     /workspaces/ORB-SLAM3-STEREO-FIXED/fomo_scripted.yaml \
     $ORB_DATASET_PATH \
     $ORB_TRAJECTORY_NAME \
     $REALTIME_FLAG
