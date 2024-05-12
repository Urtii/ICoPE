%% Get groundTruth Data
groundTruth_ = getMulRan_groundTruth('riverside1/global_pose.csv');

%% Get IMU Data
[imuData, gpsData] = getMulRan_sensor_csv_data('riverside1/xsens_imu.csv','riverside1/gps.csv',groundTruth_.time_start,groundTruth_.time_end);

%% Interpolate Ground Truth wrt. IMU and get table
groundTruth_interp = interpolate_groundTruth(imuData, groundTruth_);

%% Plot 3D Orientation

n = size(groundTruth_interp.time_d, 1);
axisLength = 100;
axisColors = ['r', 'g', 'b']; % Red for X, Green for Y, Blue for Z
axisColors_acc = ['m', 'k', 'c']; % Magenta for X, Black for Y, Cyan for Z

figure(1);
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(2);
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
for i = 1:500:n
    % Current position, orientation, and acceleration
    pos = groundTruth_interp.pos(i, :);

    % Convert quaternion to rotation matrix
    quat_wb = groundTruth_interp.quat(i, :);
    R_wb = quat2rotm(quat_wb); % Use the quat2rotm function provided earlier if needed

    acc_vec = rotatepoint(groundTruth_interp.quat(i), imuData.acc_body(i,:));
    R_acc = diag(acc_vec)*axisLength; % Use the quat2rotm function provided earlier if needed
    R_acc(3,3) = R_acc(3,3)/10;

    mag_vec = rotatepoint(groundTruth_interp.quat(i), imuData.mag_body(i,:));
    R_mag = diag(mag_vec)*axisLength;

    % Transform base axes by rotation matrix
    axes = eye(3) * axisLength;
    transformedAxes = (R_wb * axes')';

    % figure(1)
    % % Plot position
    % plot3(pos(1), pos(2), pos(3), 'ko');
    % 
    % % Draw orientation axes with RGB colors
    % for j = 1:3
    %     endpt = pos + transformedAxes(j, :);
    %     plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors(j), 'LineWidth', 2);
    % end
    % endpt = pos + R_acc(3, :);
    % plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors_acc(3), 'LineWidth', 2);
    % endpt = pos + R_acc(1, :)+R_acc(2, :);
    % plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors_acc(2), 'LineWidth', 2);

    figure(2)
    % Plot position
    plot3(pos(1), pos(2), pos(3), 'ko');

    % Draw orientation axes with RGB colors
    for j = 1:3
        endpt = pos + transformedAxes(j, :);
        % plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors(j), 'LineWidth', 2);
    end
    endpt = pos + R_mag(1, :)+R_mag(2, :);
    plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors_acc(3), 'LineWidth', 2);

end