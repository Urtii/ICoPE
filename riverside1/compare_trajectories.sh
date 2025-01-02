#!/bin/bash

# Check if the correct number of arguments is passed
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <n_to_align>"
    exit 1
fi

# Set the n_to_align parameter
N_TO_ALIGN=$1

# Folder containing trajectory files
TRAJ_FOLDER="traj_out"
GROUND_TRUTH="${TRAJ_FOLDER}/groundTruthTrajectory.txt"

# Check if the folder and ground truth file exist
if [ ! -d "$TRAJ_FOLDER" ]; then
    echo "Error: Folder '$TRAJ_FOLDER' does not exist."
    exit 1
fi

if [ ! -f "$GROUND_TRUTH" ]; then
    echo "Error: Ground truth file '$GROUND_TRUTH' does not exist."
    exit 1
fi

# List of trajectory files to compare
TRAJECTORIES=(
    "fusedTrajectory.txt"
    "globalFuseTrajectory.txt"
    "kalmanTrajectory.txt"
    "LiDARTrajectory.txt"
)

# Run evo_ape and evo_rpe for each trajectory
for TRAJ in "${TRAJECTORIES[@]}"; do
    TRAJ_FILE="${TRAJ_FOLDER}/${TRAJ}"
    APE_RESULT_ZIP="${TRAJ_FOLDER}/${TRAJ%.txt}_ape_result.zip"
    RPE_RESULT_ZIP="${TRAJ_FOLDER}/${TRAJ%.txt}_rpe_result.zip"

    if [ ! -f "$TRAJ_FILE" ]; then
        echo "Warning: Trajectory file '$TRAJ_FILE' does not exist. Skipping."
        continue
    fi

    echo "Processing APE for $TRAJ..."
    evo_ape tum "$GROUND_TRUTH" "$TRAJ_FILE" -va --n_to_align "$N_TO_ALIGN" --save_results "$APE_RESULT_ZIP"

    echo "Processing RPE for $TRAJ..."
    evo_rpe tum "$GROUND_TRUTH" "$TRAJ_FILE" -va --n_to_align "$N_TO_ALIGN" --pose_relation point_distance -u m --save_results "$RPE_RESULT_ZIP"

done

# Combine APE results into a plot and table
echo "Generating APE combined plot and table..."
evo_res ${TRAJ_FOLDER}/*_ape_result.zip --plot --save_plot "${TRAJ_FOLDER}/ape_comparison_plot.png" --save_table "${TRAJ_FOLDER}/ape_comparison_table.csv"

# Combine RPE results into a plot and table
echo "Generating RPE combined plot and table..."
evo_res ${TRAJ_FOLDER}/*_rpe_result.zip --plot --save_plot "${TRAJ_FOLDER}/rpe_comparison_plot.png" --save_table "${TRAJ_FOLDER}/rpe_comparison_table.csv"

# Completion message
echo "Comparison complete. APE and RPE results saved in '$TRAJ_FOLDER'."
