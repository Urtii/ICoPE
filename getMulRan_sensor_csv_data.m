function [imuData, gpsData] = getMulRan_sensor_csv_data(imuData_Path, gpsData_Path, T_start, T_end)
    % getMulRan_sensor_csv_data - Reads sensor data from CSV files within a specified time range.
    %
    % INPUTS:
    %   imuData_Path - File path to the IMU data CSV file.
    %   gpsData_Path - File path to the GPS data CSV file.
    %   T_start - Start time for data extraction (optional, defaults to the first timestamp).
    %   T_end - End time for data extraction (optional, defaults to the last timestamp).
    %
    % OUTPUTS:
    %   imuData - Struct containing IMU data within the specified time range.
    %   gpsData - Struct containing GPS data within the specified time range.

    %% Get imu Data
    % Extract time range from IMU data
    imuData = struct;
    imuData_table = readtable(imuData_Path, 'ReadVariableNames', false);
    imuData_csv_data = table2array(imuData_table(:, 1:17));

    % Set default values if start and end times are not provided
    if ~exist('T_start', 'var')
        T_start = uint64(imuData_csv_data(1, 1));
    end
    
    if ~exist('T_end', 'var')
        T_end = uint64(imuData_csv_data(end, 1));
    end

    % Filter data based on the specified time range
    indices = imuData_csv_data(:, 1) >= T_start & imuData_csv_data(:, 1) <= T_end;

    % Preallocate memory for imuData arrays
    imuData.time = uint64(zeros(sum(indices), 1));
    imuData.gyro_body = zeros(sum(indices), 3);
    imuData.acc_body = zeros(sum(indices), 3);
    imuData.mag_body = zeros(sum(indices), 3);

    % Populate imuData struct with relevant information
    imuData.time = uint64(imuData_csv_data(indices, 1));
    imuData.time_start = imuData.time(1);
    imuData.time_end = imuData.time(end);
    imuData.time_norm = imuData.time - T_start;
    imuData.time_d = double(imuData.time_norm / 1000) / 1e6;

    % Read and filter IMU data
    imuData.gyro_body = imuData_csv_data(indices, 9:11);
    imuData.acc_body = imuData_csv_data(indices, 12:14);
    imuData.mag_body = imuData_csv_data(indices, 15:17);
    % imuData.mag_body = [imuData.mag_body(:,2),imuData.mag_body(:,1),-imuData.mag_body(:,3)];

    % Set covariance matrices for IMU data
    imuData.acc_Cov = eye(3) * 3;
    imuData.gyro_Cov = eye(3) * 3;

    %% Get gps Data
    % Extract time range from GPS data
    gpsData = struct;
    gpsData_table = readtable(gpsData_Path, 'ReadVariableNames', false);
    gpsData_csv_data = table2array(gpsData_table(:, 1:13));

    % Filter data based on the specified time range
    indices = gpsData_csv_data(:, 1) >= T_start & gpsData_csv_data(:, 1) <= T_end;

    % Preallocate memory for gpsData arrays
    gpsData.time = uint64(zeros(sum(indices), 1));
    gpsData.LLA = zeros(sum(indices), 3);
    gpsData.Cov = zeros(sum(indices), 3, 3);

    % Populate gpsData struct with relevant information
    gpsData.time = uint64(gpsData_csv_data(indices, 1));
    gpsData.time_start = gpsData.time(1);
    gpsData.time_end = gpsData.time(end);
    gpsData.time_norm = gpsData.time - T_start;
    gpsData.time_d = double(gpsData.time_norm / 1000) / 1e6;

    % Read and filter GPS data
    gpsData.LLA = gpsData_csv_data(indices, 2:4);
    %gpsData.velocity = NaN(size(gpsData.LLA)); %This is the actual line
    %but it will be used to store LLA covariance
    gpsData.velocity = gpsData_csv_data(indices, [5 9 13]);

    % Set default value for gpsData.LLA (if needed)
    % gpsData.LLA = gpsData.LLA - gpsData.LLA(1, :);

    % Set covariance matrix for GPS data
    gpsData.Cov(:, 1, 1) = gpsData_csv_data(indices, 5);
    gpsData.Cov(:, 2, 2) = gpsData_csv_data(indices, 9);
    gpsData.Cov(:, 3, 3) = gpsData_csv_data(indices, 13);
end
