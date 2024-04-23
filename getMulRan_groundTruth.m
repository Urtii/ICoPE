function groundTruth_ = getMulRan_groundTruth(File_Path, output_length)
    % getMulRan_groundTruth - Reads ground truth data from a CSV file.
    %
    % INPUTS:
    %   File_Path - File path to the ground truth data CSV file.
    %   output_length - Number of data points to output (optional, defaults to all).
    %
    % OUTPUTS:
    %   groundTruth_ - Struct containing ground truth data.

    %% Get groundTruth Time
    groundTruth_ = struct;

    % Read time data
    groundTruth_.time = readmatrix(File_Path, 'Range', 'A:A', 'OutputType', 'uint64');

    % Set default values if output_length is not provided
    if ~exist('output_length', 'var')
        output_length = size(groundTruth_.time, 1);
    else
        groundTruth_.time = groundTruth_.time(1:output_length);
    end

    % Populate groundTruth_ struct with relevant time information
    groundTruth_.time_start = groundTruth_.time(1);
    groundTruth_.time_end = groundTruth_.time(end);
    groundTruth_.time_norm = groundTruth_.time - groundTruth_.time(1);
    groundTruth_.time_d = double(groundTruth_.time_norm / 1000) / 1e6;

    % Read and normalize position data
    groundTruth_.csv_data = csvread(File_Path, 0, 1, [0, 1, output_length - 1, 12]);
    groundTruth_.csv_data(:, 4:4:12) = groundTruth_.csv_data(:, 4:4:12) - groundTruth_.csv_data(1, 4:4:12);

    % Extract position and SE3 data
    groundTruth_.pos = groundTruth_.csv_data(:, 4:4:12);
    groundTruth_.SE3 = zeros(4, 4, output_length);
    groundTruth_.SE3(1:3,4,:) = groundTruth_.pos.';
    groundTruth_.SE3(1:3, 1:3, :) = reshape(groundTruth_.csv_data(:, [1:3 5:7 9:11])', 3, 3, output_length);
    groundTruth_.SE3(1:3, 1:3, :) = permute(groundTruth_.SE3(1:3, 1:3, :), [2 1 3]);
    groundTruth_.SE3(4, 4, :) = 1;

    % Quaternion representation
    groundTruth_.quat = quaternion(rotm2quat(groundTruth_.SE3(1:3, 1:3, :)));
    
end
