#!/bin/bash
input_path_host=/home/mbo/bigfoot-FoMo/ijrr

process_trajectory() {
    local trajectory=$1
    local output_path_host="/home/mbo/output/orbslam3/orbslam3-${trajectory}"
    for date_dir in "${input_path_host}"/*/; do
        date=$(basename "${date_dir}")
        # if [[ ${date} == "2025-11-03" ]]; then
        #     echo "Skipping ${date_dir}"
        #     continue
        # fi
        for dataset_dir in "${date_dir}${trajectory}_"*/; do
            [ -d "${dataset_dir}" ] || continue
            dataset=$(basename "${dataset_dir}")
            echo $dataset
            mkdir -p "${output_path_host}/${date}/${dataset}"
            echo "Processing dataset ${date}/${dataset}"

            log_file="${output_path_host}/${date}/${dataset}/output.log"
            echo "START_TIME: $(date +%s)" > "$log_file"

            docker run -it --rm \
                -v "${dataset_dir}":/dataset \
                -v "${dataset_dir}":/dataset \
                -v "${output_path_host}/${date}/${dataset}":/output \
                -e DATASET_PATH=/dataset \
                -e OUTPUT_PATH=/output \
                -e TRAJECTORY_NAME="${dataset}_${dataset}.txt" \
                orbslam3offline:latest run_orbslam3 \
                2>&1 | tee -a "${log_file}"
            echo "END_TIME: $(date +%s)" >> "$log_file"
        done
    done
}

process_trajectory "red" # Tuesday evening
process_trajectory "blue"
process_trajectory "orange"
process_trajectory "green"
process_trajectory "magenta"
process_trajectory "yellow"