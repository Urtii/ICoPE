#!/usr/bin/env python3

import numpy as np
import csv

def transformation_matrix_to_quaternion(matrix):
    """
    Converts a 3x4 transformation matrix to a quaternion (qw, qx, qy, qz) and translation (tx, ty, tz).
    """
    # Extract rotation matrix (top-left 3x3 part) and translation (last column)
    rotation_matrix = matrix[:3, :3]
    tx, ty, tz = matrix[:3, 3]

    # Compute quaternion from rotation matrix
    qw = np.sqrt(1 + np.trace(rotation_matrix)) / 2.0
    qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * qw)
    qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * qw)
    qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * qw)

    return (tx, ty, tz, qx, qy, qz, qw)

def convert_mulran_to_tum(input_csv, output_txt):
    """
    Converts MulRan ground truth data from CSV format to TUM format.

    Parameters:
        input_csv (str): Path to the input CSV file.
        output_txt (str): Path to the output TUM format file.
    """
    with open(input_csv, 'r') as csv_file, open(output_txt, 'w') as tum_file:
        csv_reader = csv.reader(csv_file)
        
        for row in csv_reader:
            # Parse the time and transformation matrix
            timestamp = float(row[0])
            transformation_matrix = np.array([row[1:5], row[5:9], row[9:13]], dtype=float)

            # Convert to TUM format (timestamp tx ty tz qx qy qz qw)
            tx, ty, tz, qx, qy, qz, qw = transformation_matrix_to_quaternion(transformation_matrix)

            # Write to TUM file
            tum_file.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

# Example usage
# convert_mulran_to_tum('input.csv', 'global_pose_tum.txt')
