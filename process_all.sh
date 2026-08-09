#!/bin/bash
# Process all bagfiles in fomo-visual for a self-experiment

# Without pipefail the exit status of `orbslam | tee` is tee's, so a crashed run
# looked exactly like a successful one.
set -o pipefail

# A segfault otherwise leaves nothing to debug.  Note that kernel.core_pattern pipes
# to apport, which discards cores from unpackaged binaries; to actually keep one, run
#   sudo sysctl -w kernel.core_pattern=/tmp/core.%e.%p
ulimit -c unlimited 2>/dev/null

BASE_DIR="/media/mabox/SSD_Matej/fomo-visual"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORB_REALTIME=${ORB_REALTIME:-false}

REALTIME_FLAG=""
case "${ORB_REALTIME,,}" in
    1|true|yes|on) REALTIME_FLAG="--realtime" ;;
esac

USE_LOCAL_COPY=true

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --no-local) USE_LOCAL_COPY=false ;;
        --local) USE_LOCAL_COPY=true ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

if [ ! -d "$BASE_DIR" ]; then
    echo "Error: $BASE_DIR does not exist."
    exit 1
fi

# Find all metadata.yaml files to find datasets
mapfile -t META_FILES < <(find "$BASE_DIR" -type f -name "metadata.yaml" | sort)
# mapfile -t META_FILES < <(find "$BASE_DIR" -type f -name "metadata.yaml" | grep -viE 'orange.*2025-10' | sort)

if [ ${#META_FILES[@]} -eq 0 ]; then
    echo "No sequences found in $BASE_DIR."
    exit 0
fi

echo "Available sequences:"
for i in "${!META_FILES[@]}"; do
    DATASET_PATH=$(dirname "${META_FILES[$i]}")
    TRAJECTORY_NAME=$(basename "$DATASET_PATH")
    echo "$((i+1))) $TRAJECTORY_NAME ($DATASET_PATH)"
done
echo "$(( ${#META_FILES[@]} + 1 ))) All sequences"

read -p "Choose a sequence to execute (1-$(( ${#META_FILES[@]} + 1 ))): " choice

if [[ ! "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "$(( ${#META_FILES[@]} + 1 ))" ]; then
    echo "Invalid choice."
    exit 1
fi

if [ "$choice" -eq "$(( ${#META_FILES[@]} + 1 ))" ]; then
    SELECTED_FILES=("${META_FILES[@]}")
else
    SELECTED_FILES=("${META_FILES[$((choice-1))]}")
fi

for meta_file in "${SELECTED_FILES[@]}"; do
    DATASET_PATH=$(dirname "$meta_file")
    TRAJECTORY_NAME=$(basename "$DATASET_PATH")
    # Update metadata.yaml to point to output.mcap instead of renaming the file
    if grep -q "${TRAJECTORY_NAME}.mcap" "$meta_file"; then
        echo "Updating metadata.yaml in $DATASET_PATH to use output.mcap"
        sed -i "s/${TRAJECTORY_NAME}.mcap/output.mcap/g" "$meta_file"
    fi

    # The atlas file is expected at <dataset_path>/<trajectory_name>/atlas.osa
    # ORB-SLAM3 appends .osa internally, so we provide the path without it.
    ATLAS_PATH="$DATASET_PATH/$TRAJECTORY_NAME/atlas"

    echo "=========================================================="
    echo "Processing trajectory: $TRAJECTORY_NAME"
    echo "Dataset path: $DATASET_PATH"
    echo "Atlas path: $ATLAS_PATH"
    echo "=========================================================="

    # Source ROS 2 environment (needed for the ros2 bag info check below)
    source /opt/ros/jazzy/setup.bash

    if [ "$USE_LOCAL_COPY" = true ]; then
        # Cache the sequence on the local Desktop for faster processing.  The copy
        # survives between runs and is only re-made when it is missing, holds a
        # different sequence, or no longer reads back as a valid rosbag.
        LOCAL_DATASET_PATH="$HOME/Desktop/current_sequence"
        STAMP_FILE="$LOCAL_DATASET_PATH/.cached_sequence"

        REUSE_LOCAL=false
        if [ -f "$STAMP_FILE" ] && [ "$(cat "$STAMP_FILE")" = "$TRAJECTORY_NAME" ]; then
            if ros2 bag info "$LOCAL_DATASET_PATH" >/dev/null 2>&1; then
                REUSE_LOCAL=true
            else
                echo "Local copy of $TRAJECTORY_NAME failed 'ros2 bag info'; re-copying."
            fi
        fi

        if [ "$REUSE_LOCAL" = true ]; then
            echo "Reusing existing local copy at $LOCAL_DATASET_PATH"
        else
            echo "Copying sequence to $LOCAL_DATASET_PATH..."
            rm -rf "$LOCAL_DATASET_PATH"
            mkdir -p "$LOCAL_DATASET_PATH"
            rsync -a --info=progress2 "$DATASET_PATH/" "$LOCAL_DATASET_PATH/"
            if ! ros2 bag info "$LOCAL_DATASET_PATH" >/dev/null 2>&1; then
                echo "Error: local copy at $LOCAL_DATASET_PATH is not a readable rosbag. Skipping $TRAJECTORY_NAME."
                rm -rf "$LOCAL_DATASET_PATH"
                continue
            fi
            printf '%s\n' "$TRAJECTORY_NAME" > "$STAMP_FILE"
        fi
        EXEC_DATASET_PATH="$LOCAL_DATASET_PATH"
    else
        echo "Using mounted dataset directly..."
        EXEC_DATASET_PATH="$DATASET_PATH"
    fi

    echo "--- Phase 2: Localization ---"
    OUTPUT_PATH_LOC="$DATASET_PATH/process_rosbag_localization_final"
    mkdir -p "$OUTPUT_PATH_LOC"
    cd "$OUTPUT_PATH_LOC" || exit 1

    # Update YAML for localization mode (do load, no save)
    "$WORKSPACE_DIR/configure.py" --load-atlas "$ATLAS_PATH" --save-atlas ""

    # tee truncates run.log, so keep the previous run's log as evidence instead of
    # overwriting it the moment a re-run starts.
    if [ -f "run.log" ]; then
        mv "run.log" "run.$(date -r "run.log" +%Y%m%d-%H%M%S).log"
    fi

    # Run ORB-SLAM3 offline (localization).  stdbuf -oL keeps stdout line-buffered
    # through the pipe so progress streams to the terminal instead of arriving in
    # 4KB bursts; tee keeps a full copy of the run next to the trajectories.
    stdbuf -oL "$WORKSPACE_DIR/Examples/Stereo-Inertial/stereo_inertial_fomo_ros" \
        "$WORKSPACE_DIR/Vocabulary/ORBvoc.txt" \
        "$WORKSPACE_DIR/fomo_scripted.yaml" \
        "$EXEC_DATASET_PATH" \
        "${TRAJECTORY_NAME}_loc" \
        "${REALTIME_FLAG}" 2>&1 | tee "run.log"
    ORB_STATUS=${PIPESTATUS[0]}

    # When a child dies on a signal bash reports it on its own stderr, which never
    # reaches the pipe feeding tee -- so run.log ended mid-frame with no explanation.
    # Record the outcome in the log itself.
    FRAMES_DONE=$(grep -c '^Frame' "run.log")
    if [ "$ORB_STATUS" -gt 128 ]; then
        SIG=$(( ORB_STATUS - 128 ))
        echo "ORB-SLAM3 killed by signal $SIG ($(kill -l "$SIG" 2>/dev/null)) after $FRAMES_DONE frames" | tee -a "run.log"
    elif [ "$ORB_STATUS" -ne 0 ]; then
        echo "ORB-SLAM3 exited with status $ORB_STATUS after $FRAMES_DONE frames" | tee -a "run.log"
    fi

    # Plot every exported trajectory: all segments joined, the longest one, the first one.
    source "$WORKSPACE_DIR/.venv/bin/activate"
    for seg in full longest first; do
        LOC_TRAJ_FILE="${TRAJECTORY_NAME}_loc_trajectory_${seg}.txt"
        if [ -f "$LOC_TRAJ_FILE" ]; then
            echo "Plotting ${seg} localization trajectory..."
            python3 "$WORKSPACE_DIR/plot_trajectory.py" \
                "$LOC_TRAJ_FILE" "${TRAJECTORY_NAME}_loc_trajectory_${seg}.png"
        else
            echo "WARNING: $LOC_TRAJ_FILE is missing - $TRAJECTORY_NAME did not finish. See $OUTPUT_PATH_LOC/run.log"
        fi
    done
    deactivate

    # Copy the full trajectory under a <mapping-session>_<localization-session> name, so
    # the filename records which atlas was localized against which rosbag.  The mapping
    # name is the directory holding the atlas; the localization name is the bag played.
    MAP_NAME=$(basename "$(dirname "$ATLAS_PATH")")
    FULL_TRAJ_FILE="${TRAJECTORY_NAME}_loc_trajectory_full.txt"
    if [ -f "$FULL_TRAJ_FILE" ]; then
        cp "$FULL_TRAJ_FILE" "${MAP_NAME}_${TRAJECTORY_NAME}.txt"
        echo "Copied full trajectory to ${MAP_NAME}_${TRAJECTORY_NAME}.txt"
    fi

    # The local copy is deliberately kept so the next run of the same sequence reuses it.
done
