#!/bin/bash

# Array of dataset folders and their corresponding n_to_align values
declare -A DATASETS
DATASETS=(
    [DCC2]=300
    [riverside1]=1500
    [riverside2]=1950
    [riverside3]=1050
)

# Iterate over each dataset folder and process trajectories
for DATASET in "${!DATASETS[@]}"; do
    N_TO_ALIGN="${DATASETS[$DATASET]}"
    TRAJ_FOLDER="${DATASET}/traj_out"
    GROUND_TRUTH="${TRAJ_FOLDER}/groundTruthTrajectory.txt"

    # Check if the folder and ground truth file exist
    if [ ! -d "$TRAJ_FOLDER" ]; then
        echo "Error: Folder '$TRAJ_FOLDER' does not exist. Skipping dataset '$DATASET'."
        continue
    fi

    if [ ! -f "$GROUND_TRUTH" ]; then
        echo "Error: Ground truth file '$GROUND_TRUTH' does not exist. Skipping dataset '$DATASET'."
        continue
    fi

    # List of trajectory files to compare
    TRAJECTORIES=(
        "fusedTrajectory.txt"
        "globalFuseTrajectory.txt"
        "kalmanTrajectory.txt"
        "LiDARTrajectory.txt"
        "CorrectedTrajectory.txt"
    )

    # Run evo_ape and evo_rpe for each trajectory
    for TRAJ in "${TRAJECTORIES[@]}"; do
        TRAJ_FILE="${TRAJ_FOLDER}/${TRAJ}"
        APE_RESULT_ZIP="${TRAJ_FOLDER}/${TRAJ%.txt}_ape_result.zip"
        RPE_RESULT_ZIP="${TRAJ_FOLDER}/${TRAJ%.txt}_rpe_result.zip"

        if [ ! -f "$TRAJ_FILE" ]; then
            echo "Warning: Trajectory file '$TRAJ_FILE' does not exist in dataset '$DATASET'. Skipping."
            continue
        fi

        echo "Processing APE for $TRAJ in dataset $DATASET..."
        yes | evo_ape tum "$GROUND_TRUTH" "$TRAJ_FILE" -va --n_to_align "$N_TO_ALIGN" --save_results "$APE_RESULT_ZIP"

        echo "Processing RPE for $TRAJ in dataset $DATASET..."
        yes | evo_rpe tum "$GROUND_TRUTH" "$TRAJ_FILE" -va --n_to_align "$N_TO_ALIGN" --pose_relation point_distance -u m --save_results "$RPE_RESULT_ZIP"

    done

    # Combine APE results into a plot and table
    echo "Generating APE combined plot and table for dataset $DATASET..."
    yes | evo_res $(ls ${TRAJ_FOLDER}/*_ape_result.zip | grep -v -e "LiDARTrajectory_ape_result.zip" -e "fusedTrajectory_ape_result.zip") --save_plot "${TRAJ_FOLDER}/ape_comparison_plot.png"
    yes | evo_res ${TRAJ_FOLDER}/*_ape_result.zip --save_table "${TRAJ_FOLDER}/ape_comparison_table.csv"

    # Combine RPE results into a plot and table
    echo "Generating RPE combined plot and table for dataset $DATASET..."
    yes | evo_res $(ls ${TRAJ_FOLDER}/*_rpe_result.zip | grep -v -e "LiDARTrajectory_rpe_result.zip") --save_plot "${TRAJ_FOLDER}/rpe_comparison_plot.png"
    yes | evo_res ${TRAJ_FOLDER}/*_rpe_result.zip --save_table "${TRAJ_FOLDER}/rpe_comparison_table.csv"

done

# Completion message
echo "All datasets processed. Results are saved in their respective folders."
