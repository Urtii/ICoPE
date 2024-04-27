%% Chose Dataset
clear
close all
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
LiDAR.counter = 1;

clear world_2_lidarMap_rotmat dataset


%% Interpolate Ground Truth wrt. IMU and get table
groundTruth_interp = interpolate_groundTruth(imuData, groundTruth_, 10,1);

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
kalmanTrajectory = struct;
kalmanTrajectory.Quat = quaternion.zeros(numSamples,1);
kalmanTrajectory.Point = zeros(numSamples,3);
kalmanTrajectory.Quat_Cov = zeros(numSamples,1);
kalmanTrajectory.Point_Cov = zeros(3,3,numSamples);
kalmanTrajectory.counter = 1;

numSamples = size(LiDARData,1);
fusedTrajectory = struct;
fusedTrajectory.Quat = quaternion.ones(numSamples,1);
fusedTrajectory.Point = zeros(3,numSamples);
fusedTrajectory.Quat_Cov = zeros(1,numSamples);
fusedTrajectory.Point_Cov = zeros(3,3,numSamples);
fusedTrajectory.counter = 1;
sK_mean = zeros(4,numSamples);
sL_mean = zeros(4,numSamples);

clear numSamples;

%Objects to track active pose

kalman_active = struct;
kalman_active.Pose.Point = zeros(3,1);
kalman_active.Pose.Point_Cov = eye(3,3);
kalman_active.Pose.Quat = quaternion.ones(1);
kalman_active.Pose.Quat_Cov = 1;
kalman_active.oldPose = kalman_active.Pose;
kalman_active.Twist = kalman_active.Pose;

LiDAR_active = kalman_active;
Fused_active = kalman_active;

clear sensorData LiDARData;

% Iterate the filter for prediction and correction using sensor data.

R_ned_to_enu = [0 1 0; 1 0 0; 0 0 -1];
q_ned_to_enu = quaternion(rotm2quat(R_ned_to_enu));

for ii=1:size(All_sensors,1)
    if ii ~= 1
        predict(insAsyncFilter, dt(ii-1));
    end

    % Fuse INS sensors
    insAsyncFilter = fuseINS(insAsyncFilter,All_sensors(ii,:),tuned_params);

    % Estimate Position and Orientation
    [Point, Quat] = pose(insAsyncFilter);
    covPoint = insAsyncFilter.StateCovariance(8:10,8:10);
    covQuatEst = det(insAsyncFilter.StateCovariance(1:4,1:4));

    % If INS sensors updated, record kalman pose
    if (all(~isnan(All_sensors.Accelerometer(ii,:))) || ...
       all(~isnan(All_sensors.Gyroscope(ii,:)))      || ...
       all(~isnan(All_sensors.Magnetometer(ii,:)))   || ...
       all(~isnan(All_sensors.GPSPosition(ii,:))))
        kalmanTrajectory.Point(kalmanTrajectory.counter,:) = Point*R_ned_to_enu.'; % = (R*pEst.').'
        kalmanTrajectory.Point_Cov(:,:,kalmanTrajectory.counter) = R_ned_to_enu * covPoint * R_ned_to_enu;
        kalmanTrajectory.Quat(kalmanTrajectory.counter) = q_ned_to_enu*Quat;
        kalmanTrajectory.Quat_Cov(kalmanTrajectory.counter) = covQuatEst;
        kalmanTrajectory.counter = kalmanTrajectory.counter + 1;
    end

    % If LiDAR updated, merge kalman and LiDAR estimation
    if (all(~isnan(All_sensors.Position(ii,:))))
        %Set old poses
        kalman_active.oldPose = kalman_active.Pose;
        Fused_active.oldPose = Fused_active.Pose;
        LiDAR_active.oldPose = LiDAR_active.Pose;

        %Update new Poses
        % Kalman Filter First
        kalman_active.Pose.Point = R_ned_to_enu*Point.'; % = (R*pEst.').'
        kalman_active.Pose.Point_Cov = R_ned_to_enu * covPoint * R_ned_to_enu;
        kalman_active.Pose.Quat = q_ned_to_enu*Quat;
        kalman_active.Pose.Quat_Cov = covQuatEst;

        kalman_active.Twist.Point = kalman_active.Pose.Point - kalman_active.oldPose.Point;
        kalman_active.Twist.Point = quatrotate(conj(kalman_active.oldPose.Quat),kalman_active.Twist.Point.').';
        kalman_active.Twist.Point_Cov = kalman_active.Pose.Point_Cov / kalman_active.oldPose.Point_Cov;
        kalman_active.Twist.Quat = kalman_active.Pose.Quat * conj(kalman_active.oldPose.Quat);
        kalman_active.Twist.Quat_Cov = kalman_active.Pose.Quat_Cov / kalman_active.oldPose.Quat_Cov;

        % Lidar Second
        LiDAR_active.Pose.Point = LiDAR.pos(LiDAR.counter,:).';
        LiDAR_active.Pose.Point_Cov = LiDAR.cov(LiDAR.counter)*eye(3);
        LiDAR_active.Pose.Quat = LiDAR.quat(LiDAR.counter);
        LiDAR_active.Pose.Quat_Cov = LiDAR.cov(LiDAR.counter);

        LiDAR_active.Twist.Point = LiDAR_active.Pose.Point - LiDAR_active.oldPose.Point;
        LiDAR_active.Twist.Point = quatrotate(conj(LiDAR_active.oldPose.Quat),LiDAR_active.Twist.Point.').';
        LiDAR_active.Twist.Point_Cov = LiDAR_active.Pose.Point_Cov / LiDAR_active.oldPose.Point_Cov;
        LiDAR_active.Twist.Quat = LiDAR_active.Pose.Quat * conj(LiDAR_active.oldPose.Quat);
        LiDAR_active.Twist.Quat_Cov = LiDAR_active.Pose.Quat_Cov / LiDAR_active.oldPose.Quat_Cov;

        % Calculate and record Fused Trajectory
        sK = [diag(kalman_active.Twist.Point_Cov); kalman_active.Twist.Quat_Cov];
        sL = [diag(LiDAR_active.Twist.Point_Cov); LiDAR_active.Twist.Quat_Cov];
        sK_mean(:,fusedTrajectory.counter) = sK;
        sL_mean(:,fusedTrajectory.counter) = sL;


        %K*wK + L*wL = F
        %wL+ wK = 1
        %wL = sK/(sK+SL)
        %wK = sL/(sK+sL)
        %F_body = (K*sL + L*sK)/(sK+sL) 
        %F_world = qF*F_body

        if fusedTrajectory.counter > 1
            fusedTrajectory.Point(:,fusedTrajectory.counter) = fusedTrajectory.Point(:,fusedTrajectory.counter-1) + ...
            quatrotate(fusedTrajectory.Quat(fusedTrajectory.counter-1),...
                ((kalman_active.Twist.Point.*sL(1:3) + LiDAR_active.Twist.Point.*sK(1:3))  ./  (sK(1:3)+sL(1:3))).').';
            fusedTrajectory.Quat(fusedTrajectory.counter) = slerp(kalman_active.Twist.Quat,LiDAR_active.Twist.Quat,1-(sL(4)/(sL(4)+sK(4))))*fusedTrajectory.Quat(fusedTrajectory.counter-1);
            fusedTrajectory.counter = fusedTrajectory.counter + 1;
        else
            fusedTrajectory.Point(:,fusedTrajectory.counter) = ...
                (kalman_active.Twist.Point.*sL(1:3) + LiDAR_active.Twist.Point.*sK(1:3))  ./  (sK(1:3)+sL(1:3));
            fusedTrajectory.Quat(fusedTrajectory.counter) = slerp(kalman_active.Twist.Quat,LiDAR_active.Twist.Quat,1-(sL(4)/(sL(4)+sK(4))));
            fusedTrajectory.counter = fusedTrajectory.counter + 1;
        end
        LiDAR.counter = LiDAR.counter + 1;
        
    end
end

%% Plot Position 3D

figure();
grid on
plot3(kalmanTrajectory.Point(:,1),kalmanTrajectory.Point(:,2),kalmanTrajectory.Point(:,3), 'b');
hold on
plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
plot3(LiDAR.pos(:,1),LiDAR.pos(:,2),LiDAR.pos(:,3));
plot3(fusedTrajectory.Point(1,:),fusedTrajectory.Point(2,:),fusedTrajectory.Point(3,:));
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Kalman','GroundTruth','Lidar','Weighted');

figure;
for i = 1:4
    % Create new figure
    subplot(4, 1, i); % 2 rows, 1 column, first subplot
    plot(sK_mean(i,:));
    hold on
    plot(sL_mean(i,:));
    ylim([0 3])
end


%% Error Estimation

% kalmanTrajectory.Point = kalmanTrajectory.Point*R_ned_to_enu.'; % = (R*pEst.').'
% orientationError = rad2deg(dist(kalmanTrajectory.Quat, groundTruth.Orientation));
% rmsorientationError = sqrt(mean(orientationError.^2))
% 
% positionError = sqrt(sum((kalmanTrajectory.Point - groundTruth.Position).^2, 2));
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
% plot3(kalmanTrajectory.Point(:,1),kalmanTrajectory.Point(:,2),kalmanTrajectory.Point(:,3), 'b');
% hold on
% plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
% plot3(LiDAR.pos(:,1),LiDAR.pos(:,2),LiDAR.pos(:,3));
% plot3(fusedTrajectory.Point(:,1),fusedTrajectory.Point(:,2),fusedTrajectory.Point(:,3));
% % plot3((gpsData.LLA(:,2)-gpsData.LLA(1,2))*100000,(gpsData.LLA(:,1)-gpsData.LLA(1,1))*100000,(gpsData.LLA(:,3)-gpsData.LLA(1,3)));
% title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% legend('Kalman','GroundTruth','Lidar','Weighted');

%% Plot 3D Orientation

% n = size(Point, 1);
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
% %     pos = Point(i, :);
%     quat_wb = Quat(i, :);
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