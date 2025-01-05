import os
import pandas as pd

# Mapping of trajectory names to new values
column_mappings = {
    "CorrectedTrajectory.txt": "Düzeltilmiş Ortalama",
    "LiDARTrajectory.txt": "LiDAR",
    "fusedTrajectory.txt": "Dönüşüm Ortalama",
    "globalFuseTrajectory.txt": "Global Ortalama",
    "kalmanTrajectory.txt": "Kalman Filtresi"
}

# Mapping file names to LaTeX table value
file_name_mapping = {
    "ape_comparison_table.csv": "Mutlak",
    "rpe_comparison_table.csv": "Görece"
}

# LaTeX table header template
latex_header = r"""
\begin{tabular}{|l|r|r|r|r|r|r|}
\hline
    \textbf{} & \textbf{RMSE} & \textbf{Ort.} & \textbf{Medyan} & \textbf{Std.} & \textbf{Min} & \textbf{Max} \\ \hline
"""

# Function to generate LaTeX table from a DataFrame
def generate_latex_table(df, file_path):
    latex_rows = []
    # Iterate over rows to format them in LaTeX
    for index, row in df.iterrows():
        formatted_row = " & ".join([f"\\textbf{{{row[0]}}}"] + [f"{val}" for val in row[1:]]) + r" \\ \hline"
        latex_rows.append(formatted_row)
    
    # Get file details for caption
    parent_folder = os.path.dirname(file_path)
    file_name = os.path.basename(file_path)
    value_for_caption = file_name_mapping.get(file_name, "Unknown")
    caption = f"\\caption{{{parent_folder} Veriseti için {value_for_caption} Hata Tablosu}}"

    # Combine all parts
    latex_table = latex_header + "\n".join(latex_rows) + "\n" + r"\end{tabular}" + "\n" + caption
    return latex_table

# Function to process each CSV file
def process_csv(file_path):
    # Read the CSV file
    df = pd.read_csv(file_path)

    # Change first column values
    df.iloc[:, 0] = df.iloc[:, 0].replace(column_mappings)

    # Remove the last column (assumed 8th column)
    if len(df.columns) >= 8:
        df = df.iloc[:, :-1]

    # Round all numeric values to 2 digits
    df.iloc[:, 1:] = df.iloc[:, 1:].round(2)

    # Generate LaTeX table
    latex_output = generate_latex_table(df, file_path)

    # Write the LaTeX output to a .txt file
    output_file = file_path.replace(".csv", ".txt")
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(latex_output)
    print(f"LaTeX output written to: {output_file}")

# Recursive search for CSV files
def find_and_process_csv(root_folder, target_files):
    for subdir, _, files in os.walk(root_folder):
        for file_name in files:
            if file_name in target_files:
                full_path = os.path.join(subdir, file_name)
                print(f"Processing file: {full_path}")
                process_csv(full_path)

# Define the target files
target_files = {"ape_comparison_table.csv", "rpe_comparison_table.csv"}

# Run the script
root_folder = "."  # Change to your directory path if needed
find_and_process_csv(root_folder, target_files)
