#!/bin/bash
# Process all bagfiles in ~/SSD_Matej/fomo-visual for a self-experiment

BASE_DIR="$HOME/SSD_Matej/fomo-visual"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -d "$BASE_DIR" ]; then
    echo "Error: $BASE_DIR does not exist."
    exit 1
fi

# Find all metadata.yaml files to find datasets
mapfile -t META_FILES < <(find "$BASE_DIR" -type f -name "metadata.yaml" | sort)

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
    
    # Copy sequence to local Desktop folder for faster processing
    LOCAL_DATASET_PATH="$HOME/Desktop/current_sequence"
    echo "Copying sequence to $LOCAL_DATASET_PATH..."
    rm -rf "$LOCAL_DATASET_PATH"
    mkdir -p "$LOCAL_DATASET_PATH"
    rsync -a --info=progress2 "$DATASET_PATH/" "$LOCAL_DATASET_PATH/"
    
    # Source ROS 2 environment
    source /opt/ros/jazzy/setup.bash
    
    echo "--- Phase 2: Localization ---"
    OUTPUT_PATH_LOC="$DATASET_PATH/process_rosbag_localization"
    mkdir -p "$OUTPUT_PATH_LOC"
    cd "$OUTPUT_PATH_LOC" || exit 1
    
    # Update YAML for localization mode (do load, no save)
    "$WORKSPACE_DIR/configure.py" --load-atlas "$ATLAS_PATH" --save-atlas ""
    
    # Run ORB-SLAM3 offline (localization)
    "$WORKSPACE_DIR/Examples/Stereo-Inertial/stereo_inertial_fomo_ros" \
        "$WORKSPACE_DIR/Vocabulary/ORBvoc.txt" \
        "$WORKSPACE_DIR/fomo_scripted.yaml" \
        "$LOCAL_DATASET_PATH" \
        "${TRAJECTORY_NAME}_loc" \
        "--realtime"
        
    # Plot the localization trajectory
    LOC_TRAJ_FILE="${TRAJECTORY_NAME}_loc_first_bak"
    if [ -f "$LOC_TRAJ_FILE" ]; then
        echo "Plotting localization trajectory..."
        source "$WORKSPACE_DIR/.venv/bin/activate"
        python3 "$WORKSPACE_DIR/plot_trajectory.py" "$LOC_TRAJ_FILE" "${TRAJECTORY_NAME}_loc_trajectory.png"
        deactivate
    fi
    
    # Remove the local copy
    echo "Removing local sequence copy..."
    rm -rf "$LOCAL_DATASET_PATH"
    # exit 1
done
