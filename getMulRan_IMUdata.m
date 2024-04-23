function imuData = getMulRan_IMUdata(filePath,T_start, T_end)

    % Load the bag file
    bSel_imu = select(rosbag(filePath),'Topic','/imu/data_raw');
    imuMsgs = readMessages(bSel_imu,'DataFormat','struct');
    
    imuData = struct;
    
    % Extract timestamps
    time_sec = cellfun(@(m) uint64(m.Header.Stamp.Sec), imuMsgs);
    time_nsec = cellfun(@(m) uint64(m.Header.Stamp.Nsec), imuMsgs);
    imuData.time = time_sec*1e9+time_nsec;
    
    if ~exist('T_start','var')
    % second parameter does not exist, so default it to something
        T_start = imuData.time(1);
    end
    
    if ~exist('T_end','var')
    % third parameter does not exist, so default it to something
        T_end = imuData.time(end);
    end

    indices = find(imuData.time >= T_start & imuData.time <= T_end);

    % Step 2: Extract the Data
    % Use the indices to extract the subset of time data
    imuData.time_norm = imuData.time(indices);

    % Step 3: Find First and Last Indices
    % Since 'indices' is already sorted (assuming imuData.time is sorted),
    % the first and last elements are the indices you're looking for
    imuData_time_startI = indices(1);
    imuData_time_endI = indices(end);
    imuData.time_norm = imuData.time_norm - T_start;
    imuData.time_d = double(imuData.time_norm/1000)/1e6;

    % Extract accelerometer and gyroscope data
    accelDataCell = cellfun(@(m) [double(m.LinearAcceleration.X), double(m.LinearAcceleration.Y), double(m.LinearAcceleration.Z)], imuMsgs, 'UniformOutput', false);
    gyroDataCell = cellfun(@(m) [double(m.AngularVelocity.X), double(m.AngularVelocity.Y), double(m.AngularVelocity.Z)], imuMsgs, 'UniformOutput', false);



    % Use these matrices or their averages as representative noise characteristics
    imuData.acc_Cov = reshape(imuMsgs{1}.LinearAccelerationCovariance, [3, 3]);
    imuData.gyro_Cov = reshape(imuMsgs{1}.AngularVelocityCovariance, [3, 3]);

    % Convert cell arrays to matrices
    imuData.acc_body = cell2mat(accelDataCell(imuData_time_startI:imuData_time_endI,:));
    imuData.gyro_body = cell2mat(gyroDataCell(imuData_time_startI:imuData_time_endI,:));   
    
end