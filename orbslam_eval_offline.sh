#!/bin/bash
trajectory="magenta"

input_path_host=/home/mbo/bigfoot-FoMo/ijrr
output_path_host="/home/mbo/output/orbslam3-${trajectory}"

for date_dir in "${input_path_host}"/*/; do
    date=$(basename "${date_dir}")
    for dataset_dir in "${date_dir}${trajectory}_"*/; do
        if [[ ${date} != "2025-03-10" ]]; then
            continue
        fi
        [ -d "${dataset_dir}" ] || continue
        dataset=$(basename "${dataset_dir}")
        echo $dataset
        mkdir -p "${output_path_host}/${date}/${dataset}"
        echo "Processing dataset ${date}/${dataset}"
        docker run -it --rm \
            -v "${dataset_dir}":/dataset \
            -v "${output_path_host}/${date}/${dataset}":/output \
            -e DATASET_PATH=/dataset \
            -e OUTPUT_PATH=/output \
            orbslam3offline:latest run_orbslam3 \
            2>&1 | tee "${output_path_host}/${date}/${dataset}/docker.log"
    done
done

# for date_dir in "${input_path_host}"/*/; do
#     date=$(basename "${date_dir}")
#     # Skip folders before June 2025
#     if [[ "${date}" < "2025-06" ]]; then
#         continue
#     fi
#     for dataset_dir in "${date_dir}${trajectory}_"*/; do
#         [ -d "${dataset_dir}" ] || continue
#         dataset=$(basename "${dataset_dir}")
#         mkdir -p "${output_path_host}/${date}/${dataset}"
#         echo "Processing dataset ${date}/${dataset}"
#         docker run -it --rm \
#             -v "${dataset_dir}":/dataset \
#             -v "${output_path_host}/${date}/${dataset}":/output \
#             -e DATASET_PATH=/dataset \
#             -e OUTPUT_PATH=/output \
#             orbslam3offline:latest run_orbslam3 \
#             2>&1 | tee "${output_path_host}/${date}/${dataset}/docker.log"
#     done
# done
