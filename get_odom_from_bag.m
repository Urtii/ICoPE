function lidar_odometry = get_odom_from_bag(Path_to_bag,T_start,T_end)
    % Extract odometry data from a ROS bag file for the '/integrated_to_init_with_covariance' topic.
    %
    % Inputs:
    %   - Path_to_bag: String specifying the path to the bag file
    %
    % Output:
    %   - lidar_odometry: Struct containing fields pos, cov, and time
    
    % Select the '/integrated_to_init_with_covariance' topic from the bag
    odom_bag = select(rosbag(Path_to_bag), 'Topic', '/integrated_to_init_with_covariance');
    msgs = readMessages(odom_bag, 'DataFormat', 'struct');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5555
    %%
    lidar_odometry = struct;
    lo_ = struct;
    
    % Extract timestamps
    time_sec = cellfun(@(m) uint64(m.Header.Stamp.Sec), msgs);
    time_nsec = cellfun(@(m) uint64(m.Header.Stamp.Nsec), msgs);
    lo_.time = time_sec*1e9+time_nsec;
    
    if ~exist('T_start','var')
    % second parameter does not exist, so default it to something
        T_start = lo_.time(1);
    end
    
    if ~exist('T_end','var')
    % third parameter does not exist, so default it to something
        T_end = lo_.time(end);
    end

    indices = lo_.time(:,1) >= T_start & lo_.time(:,1) <= T_end;

    % Step 2: Extract the Data
    
    % Preallocate memory for imuData arrays
    lidar_odometry.time = uint64(zeros(sum(indices), 1));
    lidar_odometry.time_d = zeros(sum(indices), 1);
    lidar_odometry.pos = zeros(sum(indices), 3);
    lidar_odometry.cov = zeros(sum(indices), 1);
    
    
    % Use the indices to extract the subset of time data
    lidar_odometry.time = lo_.time(indices);

    % Step 3: Find First and Last Indices
    % Since 'indices' is already sorted (assuming lidar_odometry.time is sorted),
    % the first and last elements are the indices you're looking for
    lidar_odometry.time = lidar_odometry.time - T_start;
    lidar_odometry.time_d = double(lidar_odometry.time/1000)/1e6;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
    %% Get Frame Transformations
    % cam to base
    camera_to_baselink_quat = quaternion([pi/2,pi/2, 0], 'euler', 'ZYX', 'frame');

    % Convert to quaternion
    map_to_camera_quat = quaternion([-pi/2,0,pi/2], 'euler', 'ZYX', 'frame');

    % Convert rotation quaternion to rotation matrix
    map_to_camera_rotm = rotmat(map_to_camera_quat, 'point');
     
    % Extract positions using cellfun for efficiency
    positions = cellfun(@(m) [m.Pose.Pose.Position.X, m.Pose.Pose.Position.Y, ...
                              m.Pose.Pose.Position.Z], msgs, 'UniformOutput', false);
    lo_.pos = vertcat(positions{:});
    lidar_odometry.pos = (map_to_camera_rotm * lo_.pos(indices,:).').';
%     lidar_odometry.pos = lo_.pos(indices,:);
    
    % Extract orientations using cellfun for efficiency
    orientations = cellfun(@(m) [m.Pose.Pose.Orientation.W, m.Pose.Pose.Orientation.X, ...
                                 m.Pose.Pose.Orientation.Y, m.Pose.Pose.Orientation.Z], ...
                                 msgs, 'UniformOutput', false);
    lo_.WXYZ = vertcat(orientations{:});
    lidar_odometry.quat = map_to_camera_quat*quatnormalize(quaternion(lo_.WXYZ(indices,:)))*camera_to_baselink_quat;
    [W,X,Y,Z] = parts(lidar_odometry.quat);
    lidar_odometry.WXYZ = [W,X,Y,Z];
    
    % Extract only the diagonal elements of the position covariance matrix
    diagonalCovariances = cellfun(@(m) m.Pose.Covariance(1).', msgs, 'UniformOutput', false);
    lo_.cov = vertcat(diagonalCovariances{:});
    lidar_odometry.cov = sqrt(lo_.cov(indices));
                                 
end
