%% Chose Dataset
clear
dataset = 'DCC2/';

%% Get groundTruth Data
groundTruth_ = getMulRan_groundTruth([dataset 'global_pose.csv'],300);

%% Get IMU Data
[imuData, gpsData] = getMulRan_sensor_csv_data([dataset 'xsens_imu.csv'],[dataset 'gps.csv'],groundTruth_.time_start,groundTruth_.time_end);

%% Get LiDAR Odometry Data and timeTable
odom = get_odom_from_bag([dataset 'clean_odom.bag'],groundTruth_.time_start,groundTruth_.time_end);
LiDARData = timetable(seconds(odom.time_d),odom.pos, odom.WXYZ, 'VariableNames', ...
                       {'Position','Orientation'});
world_2_lidarMap_rotmat = rotmat(groundTruth_.quat(1), 'point');
odom.pos_raw = odom.pos;
odom.pos = (world_2_lidarMap_rotmat * odom.pos.').';


%% Interpolate Ground Truth wrt. IMU and get table
groundTruth_interp = interpolate_groundTruth(imuData, groundTruth_);
% % groundTruth = timetable(groundTruth_interp.quat,...
% %                     groundTruth_interp.pos,'VariableNames',{'Orientation','Position'}, 'SampleRate', 100);
% groundTruth = timetable(seconds(groundTruth_interp.time_d),groundTruth_interp.quat,...
%                     groundTruth_interp.pos,'VariableNames',{'Orientation','Position'});
% 
% %% Interpolate Ground Truth wrt. LiDAR and get table
% groundTruth_interp_LiDAR = interpolate_groundTruth(odom, groundTruth_);
% % groundTruth = timetable(groundTruth_interp.quat,...
% %                     groundTruth_interp.pos,'VariableNames',{'Orientation','Position'}, 'SampleRate', 100);
% groundTruth_LiDAR = timetable(seconds(groundTruth_interp_LiDAR.time_d),groundTruth_interp_LiDAR.quat,...
%                     groundTruth_interp_LiDAR.pos,'VariableNames',{'Orientation','Position'});
%% Interpolate gpsData and get sensorData table
% gpsData_interp = interpolate_gpsData(imuData, gpsData);
% sensorData = timetable(seconds(imuData.time_d),imuData.acc_body, imuData.gyro_body,imuData.mag_body,...
%                        gpsData_interp.LLA,gpsData_interp.velocity, 'VariableNames', ...
%                        {'Accelerometer','Gyroscope','Magnetometer','GPSPosition', ...
%                        'GPSVelocity'});
sensorData_GPS = timetable(seconds(gpsData.time_d),gpsData.LLA,gpsData.velocity, ...
                        'VariableNames',{'GPSPosition','GPSVelocity'});
sensorData_imu = timetable(seconds(imuData.time_d),imuData.acc_body, imuData.gyro_body,imuData.mag_body,...
                       'VariableNames',{'Accelerometer','Gyroscope','Magnetometer'});
sensorData = synchronize(sensorData_GPS,sensorData_imu);
          
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

measNoise = tunernoise('insfilterAsync');
measNoise.AccelerometerNoise = imuData.acc_Cov(1);
measNoise.GyroscopeNoise = imuData.gyro_Cov(1);
measNoise.GPSPositionNoise = eye(3)*[mean(gpsData.Cov(1,1,:)); ...
                                     mean(gpsData.Cov(2,2,:)); ...
                                     mean(gpsData.Cov(3,3,:))];
clear W X Y Z init_state_pos init_state_vel initialState;

%% Tuner Configuration

config = tunerconfig('insfilterAsync');
config.TunableParameters = setdiff(config.TunableParameters, ...
    {'GPSPositionNoise', 'AccelerometerNoise', 'GyroscopeNoise', ...
     'GPSVelocityNoise'});
%      {'GPSPositionNoise', 'GeomagneticVectorNoise', 'MagnetometerBiasNoise'...
%       'GPSPositionNoise', 'GPSVelocityNoise', 'MagnetometerNoise'...
%       'AccelerometerBiasNoise', 'GyroscopeBiasNoise'});
config.MaxIterations = 10;
config.StepForward = 2;

%% Tune or Pass

% tuned_params = tune(insAsyncFilter,measNoise,sensorData,groundTruth,config);
% save('tunedAsyncFilter3.mat', 'imuFilter');
tuned_params = measNoise;

%% Estimate Result

All_sensors = synchronize(sensorData,LiDARData);

dt = seconds(diff(All_sensors.Time));
numSamples = size(sensorData,1);
kalmanEst = struct;
kalmanEst.qEst = quaternion.zeros(numSamples,1);
kalmanEst.posEst = zeros(numSamples,3);
kalmanEst.covEst = struct;
kalmanEst.covEst.Orient = zeros(4,4,numSamples);
kalmanEst.covEst.AngVel = zeros(3,3,numSamples);
kalmanEst.covEst.Pos = zeros(3,3,numSamples);
kalmanEst.covEst.Vel = zeros(3,3,numSamples);

WeightedEst = struct;
WeightedEst.posEst = zeros(size(LiDARData,1),3);
WeightedEst.quatEst = init_state_quat;

kalman_enu_pos_old = zeros(1,3);
kalman_enu_cov_old = eye(3);

% Iterate the filter for prediction and correction using sensor data.
GPS_counter = 1;
Kalman_counter = 1;
LiDAR_counter = 1;

R_ned_to_enu = [0 1 0; 1 0 0; 0 0 -1];

for ii=1:size(All_sensors,1)
    if ii ~= 1
        predict(insAsyncFilter, dt(ii-1));
    end
    if all(~isnan(All_sensors.Accelerometer(ii,:)))
        fuseaccel(insAsyncFilter,All_sensors.Accelerometer(ii,:), ...
            tuned_params.AccelerometerNoise);
    end
    if all(~isnan(All_sensors.Gyroscope(ii,:)))
        fusegyro(insAsyncFilter, All_sensors.Gyroscope(ii,:), ...
            tuned_params.GyroscopeNoise);
    end
    if all(~isnan(All_sensors.Magnetometer(ii,:)))
    fusemag(insAsyncFilter,All_sensors.Magnetometer(ii,:), ...
            tuned_params.MagnetometerNoise);
    end
    if all(~isnan(All_sensors.GPSPosition(ii,:)))
        
        measNoise.GPSPositionNoise = ...
                                    eye(3)*[gpsData.Cov(GPS_counter,1,1); ...
                                            gpsData.Cov(GPS_counter,2,2); ...
                                            gpsData.Cov(GPS_counter,3,3)];
        fusegps(insAsyncFilter,All_sensors.GPSPosition(ii,:), ...
                measNoise.GPSPositionNoise);
        GPS_counter = GPS_counter + 1;
    end

    [posEst, qEst] = pose(insAsyncFilter);
    covPosEst = insAsyncFilter.StateCovariance(8:10,8:10);
    covQuatEst = insAsyncFilter.StateCovariance(1:4,1:4);

    if (all(~isnan(All_sensors.Accelerometer(ii,:))) || ...
       all(~isnan(All_sensors.Gyroscope(ii,:)))      || ...
       all(~isnan(All_sensors.Magnetometer(ii,:)))   || ...
       all(~isnan(All_sensors.GPSPosition(ii,:))))
        kalmanEst.posEst(Kalman_counter,:) = posEst*R_ned_to_enu.'; % = (R*pEst.').'
        kalmanEst.covEst.Pos(:,:,Kalman_counter) = R_ned_to_enu * covPosEst * R_ned_to_enu;

        Kalman_counter = Kalman_counter + 1;
    end

    if (all(~isnan(All_sensors.Position(ii,:))))
        kalman_enu_pos = posEst*R_ned_to_enu.'; % = (R*pEst.').'
        kalman_enu_pos_diff = kalman_enu_pos - kalman_enu_pos_old;
        kalman_enu_pos_old = kalman_enu_pos;
        temp = R_ned_to_enu * covPosEst * R_ned_to_enu;
        kalman_enu_cov = temp / kalman_enu_cov_old;
        kalman_enu_cov_old = temp;

        if LiDAR_counter == 1
            lidar_diff = odom.pos(LiDAR_counter,:);

            WeightedEst.posEst(LiDAR_counter,:) = ...
                [(lidar_diff(1) / odom.cov(LiDAR_counter) + kalman_enu_pos_diff(1) / kalman_enu_cov(1,1))*odom.cov(LiDAR_counter)*kalman_enu_cov(1,1)/(odom.cov(LiDAR_counter)+kalman_enu_cov(1,1)), ...
                 (lidar_diff(2) / odom.cov(LiDAR_counter) + kalman_enu_pos_diff(2) / kalman_enu_cov(2,2))*odom.cov(LiDAR_counter)*kalman_enu_cov(2,2)/(odom.cov(LiDAR_counter)+kalman_enu_cov(2,2)), ...
                 (lidar_diff(3) / odom.cov(LiDAR_counter) + kalman_enu_pos_diff(3) / kalman_enu_cov(3,3))*odom.cov(LiDAR_counter)*kalman_enu_cov(3,3)/(odom.cov(LiDAR_counter)+kalman_enu_cov(3,3))];
        else
            lidar_diff = odom.pos(LiDAR_counter,:) - odom.pos(LiDAR_counter-1,:);

            WeightedEst.posEst(LiDAR_counter,:) = WeightedEst.posEst(LiDAR_counter-1,:) + ...
                [(lidar_diff(1) / odom.cov(LiDAR_counter) + kalman_enu_pos_diff(1) / kalman_enu_cov(1,1))*odom.cov(LiDAR_counter)*kalman_enu_cov(1,1)/(odom.cov(LiDAR_counter)+kalman_enu_cov(1,1)), ...
                 (lidar_diff(2) / odom.cov(LiDAR_counter) + kalman_enu_pos_diff(2) / kalman_enu_cov(2,2))*odom.cov(LiDAR_counter)*kalman_enu_cov(2,2)/(odom.cov(LiDAR_counter)+kalman_enu_cov(2,2)), ...
                 (lidar_diff(3) / odom.cov(LiDAR_counter) + kalman_enu_pos_diff(3) / kalman_enu_cov(3,3))*odom.cov(LiDAR_counter)*kalman_enu_cov(3,3)/(odom.cov(LiDAR_counter)+kalman_enu_cov(3,3))];
        end

        LiDAR_counter = LiDAR_counter + 1;
    end

    
%     covEst.Orient(:,:,ii) = insAsyncFilter.StateCovariance(1:4,1:4,ii);
%     covEst.AngVel(:,:,ii) = insAsyncFilter.StateCovariance(5:7,5:7,ii);
    
%     covEst.Vel(:,:,ii) = insAsyncFilter.StateCovariance(11:13,11:13,ii);
end

% kalmanEst.posEst = kalmanEst.posEst*R_ned_to_enu.'; % = (R*pEst.').'
%%
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

%% Plot Position 3D

figure();
grid on
plot3(kalmanEst.posEst(:,1),kalmanEst.posEst(:,2),kalmanEst.posEst(:,3), 'b');
hold on
plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
plot3(odom.pos(:,1),odom.pos(:,2),odom.pos(:,3));
plot3(WeightedEst.posEst(:,1),WeightedEst.posEst(:,2),WeightedEst.posEst(:,3));
% plot3((gpsData.LLA(:,2)-gpsData.LLA(1,2))*100000,(gpsData.LLA(:,1)-gpsData.LLA(1,1))*100000,(gpsData.LLA(:,3)-gpsData.LLA(1,3)));
title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Kalman','GroundTruth','Lidar','Weighted');

%% Plot Position 2D

% figure();
% grid on
% plot3(kalmanEst.posEst(:,1),kalmanEst.posEst(:,2),kalmanEst.posEst(:,3), 'b');
% hold on
% plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
% plot3(odom.pos(:,1),odom.pos(:,2),odom.pos(:,3));
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