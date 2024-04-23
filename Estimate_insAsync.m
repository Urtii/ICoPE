%% Chose Dataset
clear
dataset = 'riverside1/';

%% Get groundTruth Data
groundTruth_ = getMulRan_groundTruth([dataset 'global_pose.csv'],30000);

%% Get IMU Data
[imuData, gpsData] = getMulRan_sensor_csv_data([dataset 'xsens_imu.csv'],[dataset 'gps.csv'],groundTruth_.time_start,groundTruth_.time_end);

%% Get LiDAR Odometry Data and timeTable
LiDAR = get_odom_from_bag([dataset 'clean_odom.bag'],groundTruth_.time_start,groundTruth_.time_end);
LiDARData = timetable(seconds(LiDAR.time_d),LiDAR.pos, LiDAR.WXYZ, 'VariableNames', ...
                       {'Position','Orientation'});
world_2_lidarMap_rotmat = rotmat(groundTruth_.quat(1), 'point');
LiDAR.pos_raw = LiDAR.pos;
LiDAR.pos = (world_2_lidarMap_rotmat * LiDAR.pos.').';

clear world_2_lidarMap_rotmat dataset


%% Interpolate Ground Truth wrt. IMU and get table
groundTruth_interp = interpolate_groundTruth(imuData, groundTruth_, 10);

%% Merge sensorData table

sensorData_GPS = timetable(seconds(gpsData.time_d),gpsData.LLA,gpsData.velocity, ...
                        'VariableNames',{'GPSPosition','GPSVelocity'});
sensorData_imu = timetable(seconds(imuData.time_d),imuData.acc_body, imuData.gyro_body,imuData.mag_body,...
                       'VariableNames',{'Accelerometer','Gyroscope','Magnetometer'});
sensorData = synchronize(sensorData_GPS,sensorData_imu);

clear sensorData_GPS sensorData_imu
          
%% Init Filter

init_state_quat = groundTruth_interp.quat(1);
[W,X,Y,Z] = parts(init_state_quat);
init_state_pos = groundTruth_interp.pos(1,:);
init_state_vel = (groundTruth_.pos(2,:)-groundTruth_interp.pos(1,:)).'/(groundTruth_.time_d(2)-groundTruth_.time_d(1));
initialState = [W;X;Y;Z;0;0;0;init_state_pos.';init_state_vel;zeros(15,1)];

insAsyncFilter = insfilterAsync(...
    'ReferenceFrame', 'NED', ...
    'ReferenceLocation', gpsData.LLA(1,:), ...
    'State', initialState);

tuned_params = tunernoise('insfilterAsync');
tuned_params.AccelerometerNoise = imuData.acc_Cov(1);
tuned_params.GyroscopeNoise = imuData.gyro_Cov(1);
tuned_params.GPSPositionNoise = eye(3)*[mean(gpsData.Cov(1,1,:)); ...
                                     mean(gpsData.Cov(2,2,:)); ...
                                     mean(gpsData.Cov(3,3,:))];
clear W X Y Z init_state_pos init_state_vel initialState init_state_quat imuData groundTruth_interp;

%% Estimate Result

All_sensors = synchronize(sensorData,LiDARData);

dt = seconds(diff(All_sensors.Time));
numSamples = size(sensorData,1);

%Objects to record trajectory
kalmanEst = struct;
kalmanEst.qEst = quaternion.zeros(numSamples,1);
kalmanEst.posEst = zeros(numSamples,3);
kalmanEst.covEst = struct;
kalmanEst.covEst.Orient = zeros(4,4,numSamples);
kalmanEst.covEst.AngVel = zeros(3,3,numSamples);
kalmanEst.covEst.Pos = zeros(3,3,numSamples);
kalmanEst.covEst.Vel = zeros(3,3,numSamples);
clear numSamples;

WeightedEst = struct;
WeightedEst.posEst = zeros(size(LiDARData,1),3);

%Objects to track active pose

kalman_body = struct;
kalman_body.Pose.Point = zeros(3,1);
kalman_body.Pose.Point_Cov = zeros(3,3);
kalman_body.Pose.Quat = quaternion(rotm2quat(eye(3)));
kalman_body.Pose.Quat_Cov = 0;
kalman_body.oldPose = kalman_body.Pose;

LiDAR_body = kalman_body;
Fused_body = kalman_body;

kalman_enu_pos_old = zeros(1,3);
kalman_enu_cov_old = eye(3);

kalman_enu_quat_old = quaternion(rotm2quat(eye(3)));
kalman_quat_cov_old = 1;


clear sensorData LiDARData;

% Iterate the filter for prediction and correction using sensor data.
Kalman_counter = 1;
LiDAR_counter = 1;

R_ned_to_enu = [0 1 0; 1 0 0; 0 0 -1];
q_ned_to_enu = quaternion(rotm2quat(R_ned_to_enu));

for ii=1:size(All_sensors,1)
    if ii ~= 1
        predict(insAsyncFilter, dt(ii-1));
    end

    % Fuse INS sensors
    insAsyncFilter = fuseINS(insAsyncFilter,All_sensors(ii,:),tuned_params);

    % Estimate Position and Orientation
    [posEst, qEst] = pose(insAsyncFilter);
    covPosEst = insAsyncFilter.StateCovariance(8:10,8:10);
    covQuatEst = det(insAsyncFilter.StateCovariance(1:4,1:4));

    % If INS sensors updated, record kalman prediction
    if (all(~isnan(All_sensors.Accelerometer(ii,:))) || ...
       all(~isnan(All_sensors.Gyroscope(ii,:)))      || ...
       all(~isnan(All_sensors.Magnetometer(ii,:)))   || ...
       all(~isnan(All_sensors.GPSPosition(ii,:))))
        kalmanEst.posEst(Kalman_counter,:) = posEst*R_ned_to_enu.'; % = (R*pEst.').'
        kalmanEst.covEst.Pos(:,:,Kalman_counter) = R_ned_to_enu * covPosEst * R_ned_to_enu;

        Kalman_counter = Kalman_counter + 1;
    end

    % If LiDAR updated, merge kalman and LiDAR estimation
    if (all(~isnan(All_sensors.Position(ii,:))))
        kalman_enu_pos = posEst*R_ned_to_enu.'; % = (R*pEst.').'
        kalman_enu_pos_diff = kalman_enu_pos - kalman_enu_pos_old;
        kalman_enu_pos_old = kalman_enu_pos;
        temp = R_ned_to_enu * covPosEst * R_ned_to_enu;
        kalman_enu_cov = temp / kalman_enu_cov_old;
        kalman_enu_cov_old = temp;

        % kalman_enu_quat = q_ned_to_enu*qEst;
        % kalman_enu_quat_diff = kalman_enu_quat * conj(kalman_enu_quat_old);
        % kalman_enu_quat_old = kalman_enu_quat;
        % kalman_quat_cov = covQuatEst / kalman_enu_cov_old;
        % kalman_enu_cov_old = covQuatEst;

        if LiDAR_counter == 1
            lidar_diff = LiDAR.pos(LiDAR_counter,:);

            WeightedEst.posEst(LiDAR_counter,:) = ...
                [(lidar_diff(1) / LiDAR.cov(LiDAR_counter) + kalman_enu_pos_diff(1) / kalman_enu_cov(1,1))*LiDAR.cov(LiDAR_counter)*kalman_enu_cov(1,1)/(LiDAR.cov(LiDAR_counter)+kalman_enu_cov(1,1)), ...
                 (lidar_diff(2) / LiDAR.cov(LiDAR_counter) + kalman_enu_pos_diff(2) / kalman_enu_cov(2,2))*LiDAR.cov(LiDAR_counter)*kalman_enu_cov(2,2)/(LiDAR.cov(LiDAR_counter)+kalman_enu_cov(2,2)), ...
                 (lidar_diff(3) / LiDAR.cov(LiDAR_counter) + kalman_enu_pos_diff(3) / kalman_enu_cov(3,3))*LiDAR.cov(LiDAR_counter)*kalman_enu_cov(3,3)/(LiDAR.cov(LiDAR_counter)+kalman_enu_cov(3,3))];
        else
            lidar_diff = LiDAR.pos(LiDAR_counter,:) - LiDAR.pos(LiDAR_counter-1,:);

            WeightedEst.posEst(LiDAR_counter,:) = WeightedEst.posEst(LiDAR_counter-1,:) + ...
                [(lidar_diff(1) / LiDAR.cov(LiDAR_counter) + kalman_enu_pos_diff(1) / kalman_enu_cov(1,1))*LiDAR.cov(LiDAR_counter)*kalman_enu_cov(1,1)/(LiDAR.cov(LiDAR_counter)+kalman_enu_cov(1,1)), ...
                 (lidar_diff(2) / LiDAR.cov(LiDAR_counter) + kalman_enu_pos_diff(2) / kalman_enu_cov(2,2))*LiDAR.cov(LiDAR_counter)*kalman_enu_cov(2,2)/(LiDAR.cov(LiDAR_counter)+kalman_enu_cov(2,2)), ...
                 (lidar_diff(3) / LiDAR.cov(LiDAR_counter) + kalman_enu_pos_diff(3) / kalman_enu_cov(3,3))*LiDAR.cov(LiDAR_counter)*kalman_enu_cov(3,3)/(LiDAR.cov(LiDAR_counter)+kalman_enu_cov(3,3))];
        end

        LiDAR_counter = LiDAR_counter + 1;
    end
end

%% Plot Position 3D

figure();
grid on
plot3(kalmanEst.posEst(:,1),kalmanEst.posEst(:,2),kalmanEst.posEst(:,3), 'b');
hold on
plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
plot3(LiDAR.pos(:,1),LiDAR.pos(:,2),LiDAR.pos(:,3));
plot3(WeightedEst.posEst(:,1),WeightedEst.posEst(:,2),WeightedEst.posEst(:,3));
% plot3((gpsData.LLA(:,2)-gpsData.LLA(1,2))*100000,(gpsData.LLA(:,1)-gpsData.LLA(1,1))*100000,(gpsData.LLA(:,3)-gpsData.LLA(1,3)));
title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Kalman','GroundTruth','Lidar','Weighted');


%% Error Estimation

% kalmanEst.posEst = kalmanEst.posEst*R_ned_to_enu.'; % = (R*pEst.').'
% orientationError = rad2deg(dist(kalmanEst.qEst, groundTruth.Orientation));
% rmsorientationError = sqrt(mean(orientationError.^2))
% 
% positionError = sqrt(sum((kalmanEst.posEst - groundTruth.Position).^2, 2));
% rmspositionError = sqrt(mean( positionError.^2))

%% Plot RMS Error
% 
% figure();
% t = (0:numSamples-1)./ groundTruth.Properties.SampleRate;
% subplot(2,1,1)
% plot(t, positionError, 'b');
% title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
% xlabel('Time (s)');
% ylabel('Position Error (meters)')
% subplot(2,1,2)
% plot(t, orientationError, 'b');
% title("Orientation Error")
% xlabel('Time (s)');
% ylabel('Orientation Error (degrees)');


%% Plot Position 2D

% figure();
% grid on
% plot3(kalmanEst.posEst(:,1),kalmanEst.posEst(:,2),kalmanEst.posEst(:,3), 'b');
% hold on
% plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
% plot3(LiDAR.pos(:,1),LiDAR.pos(:,2),LiDAR.pos(:,3));
% plot3(WeightedEst.posEst(:,1),WeightedEst.posEst(:,2),WeightedEst.posEst(:,3));
% % plot3((gpsData.LLA(:,2)-gpsData.LLA(1,2))*100000,(gpsData.LLA(:,1)-gpsData.LLA(1,1))*100000,(gpsData.LLA(:,3)-gpsData.LLA(1,3)));
% title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% legend('Kalman','GroundTruth','Lidar','Weighted');

%% Plot 3D Orientation

% n = size(posEst, 1);
% axisLength = 20;
% axisColors = ['r', 'g', 'b']; % Red for X, Green for Y, Blue for Z
% axisColors_acc = ['m', 'k', 'c']; % Magenta for X, Black for Y, Cyan for Z
% 
% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% for i = 1:100:n/10
%     % Current position, orientation, and acceleration
% %     pos = posEst(i, :);
%     quat_wb = qEst(i, :);
%     acc_vec = rotateframe(groundTruth_interp.quat(i), imuData.acc_body(i,:));
%     pos = groundTruth_interp.pos(i, :);
% %     quat_wb = groundTruth_interp.quat(i, :);
%     
%     % Convert quaternion to rotation matrix
%     R_wb = quat2rotm(quat_wb); % Use the quat2rotm function provided earlier if needed
%     
%     % Define base axes
%     axes = eye(3) * axisLength;
%     R_acc = diag(acc_vec)*axisLength; % Use the quat2rotm function provided earlier if needed
%     R_acc(3,3) = R_acc(3,3)/10;
% %     axes = diag(accDirBody_wog)*axisLength;
% %     norm(accDirWorld_wog)
% %     axes(9) = 10 * axes(9);
%     
%     % Transform base axes by rotation matrix
%     transformedAxes = (R_wb * axes')';
%     
%     % Plot position
%     plot3(pos(1), pos(2), pos(3), 'ko');
%     
%     % Draw orientation axes with RGB colors
%     for j = 1:3
%         endpt = pos + transformedAxes(j, :);
%         plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors(j), 'LineWidth', 2);
%         endpt = pos + R_acc(j, :);
%         plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors_acc(j), 'LineWidth', 2);
%     end
%     
% end