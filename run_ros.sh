#!/bin/bash
# Run ORB-SLAM3 on a stereo-inertial dataset

# ORB_TRAJECTORY_NAME=${ORB_TRAJECTORY_NAME:-"red_2025-06-26-10-31"}
# ORB_OUTPUT_PATH=${ORB_OUTPUT_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-08-20-10-42/process_june_rosbag"}
# ORB_LOAD_ATLAS=${ORB_LOAD_ATLAS:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/red_2025-08-20-10-42/atlas"}
# ORB_OUTPUT_PATH=${ORB_OUTPUT_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/process_june_rosbag"}
# ORB_LOAD_ATLAS=${ORB_LOAD_ATLAS:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/atlas"}

# ORB_DATASET_PATH=${ORB_DATASET_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-01-30-09-07/"}
ORB_DATASET_PATH=${ORB_DATASET_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-10-14-12-46/"}
# ORB_DATASET_PATH=${ORB_DATASET_PATH:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-06-26-10-48/"}
ORB_LOAD_ATLAS=${ORB_LOAD_ATLAS:-"/workspaces/ORB-SLAM3-STEREO-FIXED/data/orange_2025-10-14-12-46/orange_2025-10-14-12-46/atlas"}

ORB_TRAJECTORY_NAME=${ORB_TRAJECTORY_NAME:-"$(basename "${ORB_DATASET_PATH%/}")"}
ORB_OUTPUT_PATH=${ORB_OUTPUT_PATH:-"$(dirname "${ORB_LOAD_ATLAS%/}")/mapping-vo"}

ORB_PATH="/workspaces/ORB-SLAM3-STEREO-FIXED"

export ORB_REALTIME=${ORB_REALTIME:-false}

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

source .venv/bin/activate
# Update the YAML configuration
# $ORB_PATH/configure.py --load-atlas "$ORB_LOAD_ATLAS"
python configure.py --save-atlas "$ORB_LOAD_ATLAS"
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Run ORB-SLAM3
mkdir -p $ORB_OUTPUT_PATH
cd $ORB_OUTPUT_PATH
stdbuf -oL $ORB_PATH/Examples/Stereo-Inertial/stereo_inertial_fomo_ros \
     $ORB_PATH/Vocabulary/ORBvoc.txt \
     $ORB_PATH/fomo_scripted.yaml \
     $ORB_DATASET_PATH \
     $ORB_TRAJECTORY_NAME \
     $REALTIME_FLAG \
     2>&1 | tee "run.log"

# $? is tee's, which is 0 even when the SLAM binary segfaults or is OOM-killed.  Without this
# an aborted run -- one that never reached the atlas save -- still printed a success message.
ORB_EXIT=${PIPESTATUS[0]}
if [ "$ORB_EXIT" -ne 0 ]; then
    echo ""
    echo "*** ORB-SLAM3 exited abnormally (status ${ORB_EXIT}). The atlas was NOT saved. ***"
    echo "*** See run.log; check 'dmesg' for an OOM kill before assuming a crash.        ***"
    exit "$ORB_EXIT"
fi

# Copy the full trajectory under a <mapping-session>_<localization-session> name, so the
# filename records which atlas was localized against which rosbag.  The mapping name is
# the directory holding the atlas; the localization name is the bag played.
MAP_NAME=$(basename "$(dirname "${ORB_LOAD_ATLAS%/}")")
FULL_TRAJ_FILE="${ORB_TRAJECTORY_NAME}_trajectory_full.txt"
if [ -f "$FULL_TRAJ_FILE" ]; then
    cp "$FULL_TRAJ_FILE" "${MAP_NAME}_${ORB_TRAJECTORY_NAME}.txt"
    echo "Copied full trajectory to ${MAP_NAME}_${ORB_TRAJECTORY_NAME}.txt"
fi
