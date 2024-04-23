function lidarData = getMulRan_LiDARdata(filePath,T_start, T_end)

    % Load the bag file
    bSel_lidar = select(rosbag(filePath),'Topic','/integrated_to_init_with_covariance');
    lidarMsgs = readMessages(bSel_lidar,'DataFormat','struct');
    
    lidarData = struct;
    
    % Extract timestamps
    time_sec = cellfun(@(m) uint64(m.Header.Stamp.Sec), lidarMsgs);
    time_nsec = cellfun(@(m) uint64(m.Header.Stamp.Nsec), lidarMsgs);
    lidarData.time = time_sec*1e9+time_nsec;
    
    if ~exist('T_start','var')
    % second parameter does not exist, so default it to something
        T_start = lidarData.time(1);
    end
    
    if ~exist('T_end','var')
    % third parameter does not exist, so default it to something
        T_end = lidarData.time(end);
    end

    indices = find(lidarData.time >= T_start & lidarData.time <= T_end);

    % Step 2: Extract the Data
    % Use the indices to extract the subset of time data
    lidarData.time_norm = lidarData.time(indices);

    % Step 3: Find First and Last Indices
    % Since 'indices' is already sorted (assuming imuData.time is sorted),
    % the first and last elements are the indices you're looking for
    lidarData_time_startI = indices(1);
    lidarData_time_endI = indices(end);
    lidarData.time_norm = lidarData.time_norm - T_start;
    lidarData.time_d = double(lidarData.time_norm/1000)/1e6;

    % Extract accelerometer and gyroscope data
    accelDataCell = cellfun(@(m) [double(m.LinearAcceleration.X), double(m.LinearAcceleration.Y), double(m.LinearAcceleration.Z)], lidarMsgs, 'UniformOutput', false);
    gyroDataCell = cellfun(@(m) [double(m.AngularVelocity.X), double(m.AngularVelocity.Y), double(m.AngularVelocity.Z)], lidarMsgs, 'UniformOutput', false);



    % Use these matrices or their averages as representative noise characteristics
    lidarData.acc_Cov = reshape(lidarMsgs{1}.LinearAccelerationCovariance, [3, 3]);
    lidarData.gyro_Cov = reshape(lidarMsgs{1}.AngularVelocityCovariance, [3, 3]);

    % Convert cell arrays to matrices
    lidarData.acc_body = cell2mat(accelDataCell(lidarData_time_startI:lidarData_time_endI,:));
    lidarData.gyro_body = cell2mat(gyroDataCell(lidarData_time_startI:lidarData_time_endI,:));   
    
end