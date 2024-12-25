
%% Practices to maintain code
% All measurement data will be inserted in All_sensors timetable
% INS Frame of Allsensors table is NED
% Ground Truth Frame of Allsensors table is ENU
% LiDAR Frame of Allsensors table is ENU
% Algorithm only take measurement from All_sensors timetable
% All *Trajectory objects have the same template
% All *Trajectory objects will be updated only after Sensor Fusion phase
% All *_active objects have the same template and updated durinf SF phase


%% Chose Dataset
clear
close all
dataset = 'riverside2/';

%% Get groundTruth Data
groundTruth_ = getMulRan_groundTruth([dataset 'global_pose.csv']);

%% Get IMU Data
[imuData, gpsData] = getMulRan_sensor_csv_data([dataset 'xsens_imu.csv'],[dataset 'gps.csv'],groundTruth_.time_start,groundTruth_.time_end);

%% Get LiDAR Odometry Data and timeTable
LiDAR = get_odom_from_bag([dataset 'clean_odom.bag'],groundTruth_.time_start,groundTruth_.time_end);

%% Interpolate Ground Truth wrt. IMU, LiDAR and get table
groundTruth_interp = interpolate_groundTruth(imuData, groundTruth_, 10,1);
groundTruth_interp_LiDAR = interpolate_groundTruth(LiDAR,groundTruth_,0,1);
groundTruthData = timetable(seconds(groundTruth_interp_LiDAR.time_d),...
            groundTruth_interp_LiDAR.pos, groundTruth_interp_LiDAR.WXYZ,...
            'VariableNames', {'GT_Position','GT_Orientation'});

world_2_lidarMap_rotmat = rotmat(groundTruth_interp_LiDAR.quat(1)*conj(LiDAR.quat(1)), 'point');
LiDAR.pos = (world_2_lidarMap_rotmat * LiDAR.pos.').';
LiDAR.pos = LiDAR.pos - LiDAR.pos(1,:) + groundTruth_interp_LiDAR.pos(1,:);
LiDAR.quat = groundTruth_interp_LiDAR.quat(1)*conj(LiDAR.quat(1))*LiDAR.quat;
[W,X,Y,Z] = parts(LiDAR.quat);
LiDAR.WXYZ = [W,X,Y,Z];
LiDARData = timetable(seconds(LiDAR.time_d),LiDAR.pos, LiDAR.WXYZ, LiDAR.cov, 'VariableNames', ...
                       {'LiDAR_Position','LiDAR_Orientation', 'LiDAR_Cov'});
numSamples = size(LiDARData,1);

clear world_2_lidarMap_rotmat dataset W X Y Z

%% Merge Timetable

sensorData_GPS = timetable(seconds(gpsData.time_d),gpsData.LLA,gpsData.velocity, ...
                        'VariableNames',{'GPSPosition','GPSVelocity'});
sensorData_imu = timetable(seconds(imuData.time_d),imuData.acc_body, imuData.gyro_body,imuData.mag_body,...
                       'VariableNames',{'Accelerometer','Gyroscope','Magnetometer'});
All_sensors = synchronize(sensorData_GPS,sensorData_imu,LiDARData,groundTruthData);
          
%% Init Filter

R_ned_from_enu = [0 1 0; 1 0 0; 0 0 -1];
q_ned_from_enu = quaternion(rotm2quat(R_ned_from_enu));

init_state_quat = groundTruth_interp.quat(1);
[W,X,Y,Z] = parts(init_state_quat);
init_state_pos = groundTruth_interp.pos(1,:);
init_state_vel = (groundTruth_.pos(2,:)-groundTruth_interp.pos(1,:)).'/(groundTruth_.time_d(2)-groundTruth_interp.time_d(1));
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

clear sensorData_GPS sensorData_imu LiDARData groundTruthData gpsData ...
    groundTruth_interp_LiDAR imuData LiDAR
clear W X Y Z init_state_vel initialState imuData groundTruth_interp;

%% Estimate Result

dt = diff(seconds(All_sensors.Time));

%Objects to record trajectory

fusedTrajectory = struct;
fusedTrajectory.Time = zeros(1,numSamples);
fusedTrajectory.Quat = quaternion.zeros(numSamples,1);
fusedTrajectory.Point = zeros(3,numSamples);
fusedTrajectory.Quat_Cov = zeros(1,numSamples);
fusedTrajectory.Point_Cov = zeros(3,3,numSamples);
fusedTrajectory.counter = 1;

kalmanTrajectory = fusedTrajectory;
groundTruthTrajectory = fusedTrajectory;
LiDARTrajectory = fusedTrajectory;

clear numSamples;

%Objects to track active pose

kalman_active = struct;
kalman_active.Pose.Point = zeros(3,1);
kalman_active.Pose.Point_Cov = eye(3,3);
kalman_active.Pose.Quat = quaternion.ones(1);
kalman_active.Pose.Quat_Cov = 1;
       
kalman_active.Pose.Vel_Cov = zeros(3); % Current position covariance matrix
kalman_active.Pose.Acc_Cov = zeros(3); % Current position covariance matrix
kalman_active.Pose.angvel_Cov = 0;

kalman_active.oldPose = kalman_active.Pose;
kalman_active.Twist = kalman_active.Pose;

LiDAR_active = kalman_active;
fused_active = kalman_active;
fused_active.Pose.Point = zeros(3,1);
fused_active.Pose.Quat = quaternion.ones(1);

clear sensorData LiDARData;


%% Algorithm

for ii=1:size(All_sensors,1)
    if ii ~= 1
        predict(insAsyncFilter, dt(ii-1));
    end

        % If INS sensors updated, update kalman filter
    if (all(~isnan(All_sensors.Accelerometer(ii,:))) || ...
       all(~isnan(All_sensors.Gyroscope(ii,:)))      || ...
       all(~isnan(All_sensors.Magnetometer(ii,:)))   || ...
       all(~isnan(All_sensors.GPSPosition(ii,:))))
        % Fuse INS sensors
        insAsyncFilter = fuseINS(insAsyncFilter,All_sensors(ii,:),tuned_params);
    end

    [kalman_Point, kalman_Quat] = pose(insAsyncFilter);

    % If LiDAR updated, merge kalman and LiDAR estimation
    if (all(~isnan(All_sensors.LiDAR_Position(ii,:))))

        Counter = groundTruthTrajectory.counter;

        %Set old poses
        kalman_active.oldPose = kalman_active.Pose;
        fused_active.oldPose = fused_active.Pose;
        LiDAR_active.oldPose = LiDAR_active.Pose;

        if Counter == 1

            % In first iteration align sensors with Ground Truth
            fused_active.Pose.Point = All_sensors.GT_Position(ii,:).';
            % fused_active.Pose.Quat = quaternion(All_sensors.GT_Orientation(ii,:));
            fused_active.Pose.Quat = conj(q_ned_from_enu)*kalman_Quat;

            init_state_quat = q_ned_from_enu*fused_active.Pose.Quat;
            [W,X,Y,Z] = parts(init_state_quat);
            init_state_pos = R_ned_from_enu*fused_active.Pose.Point;
            init_state_vel = init_state_pos/seconds(All_sensors.Time(7));
            initialState = [W;X;Y;Z;0;0;0;init_state_pos;init_state_vel;zeros(15,1)];

            correct(insAsyncFilter,1:1:28,initialState,insAsyncFilter.StateCovariance);


            [kalman_Point, kalman_Quat] = pose(insAsyncFilter);
            kalman_active.Pose.Point = R_ned_from_enu*kalman_Point.';
            kalman_active.Pose.Point_Cov = R_ned_from_enu*insAsyncFilter.StateCovariance(8:10,8:10)*R_ned_from_enu.';
            kalman_active.Pose.Vel_Cov = R_ned_from_enu*insAsyncFilter.StateCovariance(11:13,11:13)*R_ned_from_enu.';
            kalman_active.Pose.Acc_Cov = R_ned_from_enu*insAsyncFilter.StateCovariance(14:16,14:16)*R_ned_from_enu.';
            kalman_active.Pose.Quat = conj(q_ned_from_enu)*kalman_Quat;
            kalman_active.Pose.Quat_Cov = det(insAsyncFilter.StateCovariance(1:4,1:4));
            kalman_active.Pose.angvel_Cov = det(insAsyncFilter.StateCovariance(5:7,5:7));
            kalman_active.Pose.Time = seconds(All_sensors.Time(ii));

            kalman_active = get_body_transform(kalman_active);

            % Lidar Second
            LiDAR_active.Pose.Point = All_sensors.LiDAR_Position(ii,:).';
            LiDAR_active.Pose.Point_Cov = All_sensors.LiDAR_Cov(ii,:)*eye(3);
            LiDAR_active.Pose.Quat = quaternion(All_sensors.LiDAR_Orientation(ii,:));
            LiDAR_active.Pose.Quat_Cov = All_sensors.LiDAR_Cov(ii,:);
            LiDAR_active.Pose.Time = seconds(All_sensors.Time(ii));

            LiDAR_active = get_body_transform(LiDAR_active);

        elseif Counter > 1

            % Estimate Position and Orientation
            kalman_active.Pose.Point = R_ned_from_enu*kalman_Point.';
            kalman_active.Pose.Point_Cov = R_ned_from_enu*insAsyncFilter.StateCovariance(8:10,8:10)*R_ned_from_enu.';
            kalman_active.Pose.Vel_Cov = R_ned_from_enu*insAsyncFilter.StateCovariance(11:13,11:13)*R_ned_from_enu.';
            kalman_active.Pose.Acc_Cov = R_ned_from_enu*insAsyncFilter.StateCovariance(14:16,14:16)*R_ned_from_enu.';
            kalman_active.Pose.Quat = conj(q_ned_from_enu) * kalman_Quat;
            kalman_active.Pose.Quat_Cov = det(insAsyncFilter.StateCovariance(1:4,1:4));
            kalman_active.Pose.angvel_Cov = det(insAsyncFilter.StateCovariance(5:7,5:7));
            kalman_active.Pose.Time = seconds(All_sensors.Time(ii));

            kalman_active = get_body_transform(kalman_active,dt(ii-1));

            % Lidar Second
            LiDAR_active.Pose.Point = All_sensors.LiDAR_Position(ii,:).';
            LiDAR_active.Pose.Point_Cov = All_sensors.LiDAR_Cov(ii,:)*eye(3);
            LiDAR_active.Pose.Vel_Cov = zeros(3);
            LiDAR_active.Pose.Acc_Cov = zeros(3);
            LiDAR_active.Pose.Quat = quaternion(All_sensors.LiDAR_Orientation(ii,:));
            LiDAR_active.Pose.Quat_Cov = All_sensors.LiDAR_Cov(ii,:);
            LiDAR_active.Pose.angvel_Cov = 0;
            LiDAR_active.Pose.Time = seconds(All_sensors.Time(ii));

            LiDAR_active = get_body_transform(LiDAR_active,dt(ii-1));

            % Calculate and record Fused Position and Orientation
            % sL = [1 1 1 1];
            % sK = [0 0 0 0];
            % sK = [diag(kalman_active.Twist.Point_Cov); 0];
            % sL = [diag(LiDAR_active.Twist.Point_Cov); 1];
            sK = [diag(kalman_active.Twist.Point_Cov); kalman_active.Twist.Quat_Cov];
            sL = [diag(LiDAR_active.Twist.Point_Cov); LiDAR_active.Twist.Quat_Cov];
            % for i = 1:4
            %     % if (sK(i)<0.5); sK(i) = 0.5; end
            %     % if (sL(i)<0.5); sL(i) = 0.5; end
            %     % if (sK(i)>2); sK(i) = 2; end
            %     % if (sL(i)>2); sL(i) = 2; end
            %     sK(i) = 1;
            %     sL(i) = 0;
            % end

            %K*wK + L*wL = F
            %wL+ wK = 1
            %wL = sK/(sK+SL)
            %wK = sL/(sK+sL)
            %F_body = (K*sL + L*sK)/(sK+sL)
            %F_world = qF*F_body
            
            % *Inverse Covariance Sensor Fusion*
            Fused_R_body_from_world = quat2rotm(fused_active.oldPose.Quat);
            fused_active.Pose.Point = fused_active.oldPose.Point + ...
                Fused_R_body_from_world * ((diag(sK(1:3)+sL(1:3))) \ ...
                ((diag(sL(1:3))*kalman_active.Twist.Point) + (diag(sK(1:3))*LiDAR_active.Twist.Point)));
            fused_active.Pose.Quat = fused_active.oldPose.Quat * ...
                slerp(kalman_active.Twist.Quat,LiDAR_active.Twist.Quat,1-min(max(sL(4)/(sL(4)+sK(4)),0),1));

        end
        
        % Record Trajectories
        Counter = groundTruthTrajectory.counter;
        groundTruthTrajectory.Point(:,Counter) = All_sensors.GT_Position(ii,:).';
        groundTruthTrajectory.Quat(Counter) = quaternion(All_sensors.GT_Orientation(ii,:));
        groundTruthTrajectory.Time(Counter) = seconds(All_sensors.Time(ii));

        kalmanTrajectory.Point(:,Counter) = kalman_active.Pose.Point;
        kalmanTrajectory.Point_Cov(:,:,Counter) = kalman_active.Pose.Point_Cov;
        kalmanTrajectory.Quat(Counter) = kalman_active.Pose.Quat;
        kalmanTrajectory.Quat_Cov(Counter) = kalman_active.Pose.Quat_Cov;
        kalmanTrajectory.Time(Counter) = seconds(All_sensors.Time(ii));

        LiDARTrajectory.Point(:,Counter) = LiDAR_active.Pose.Point;
        LiDARTrajectory.Point_Cov(:,:,Counter) = LiDAR_active.Pose.Point_Cov;
        LiDARTrajectory.Quat(Counter) = LiDAR_active.Pose.Quat;
        LiDARTrajectory.Quat_Cov(Counter) = LiDAR_active.Pose.Quat_Cov;
        LiDARTrajectory.Time(Counter) = seconds(All_sensors.Time(ii));

        fusedTrajectory.Point(:,Counter) = fused_active.Pose.Point;
        % fusedTrajectory.Point_Cov(:,Counter) = fused_active.Pose.Point_Cov;
        fusedTrajectory.Quat(Counter) = fused_active.Pose.Quat;
        % fusedTrajectory.Quat_Cov(Counter) = fused_active.Pose.Quat_Cov;
        fusedTrajectory.Time(Counter) = seconds(All_sensors.Time(ii));


        groundTruthTrajectory.counter = groundTruthTrajectory.counter + 1;
        kalmanTrajectory.counter = kalmanTrajectory.counter + 1;
        LiDARTrajectory.counter = LiDARTrajectory.counter + 1;
        fusedTrajectory.counter = fusedTrajectory.counter + 1;

    end
end

%% Plot Position 3D
%

% close all
% ff = figure();
% 
% % Plot each trajectory on the same figure with orientations
% step_size = 100;
% vectorLength = 1;
% plot_trajectory_with_orientation(ff, kalmanTrajectory, step_size, vectorLength, 'Kalman', 'b');
% hold on;
% plot_trajectory_with_orientation(ff, groundTruthTrajectory, step_size, vectorLength, 'Ground Truth', 'r');
% hold on;
% plot_trajectory_with_orientation(ff, LiDARTrajectory, step_size, vectorLength, 'Lidar', 'g');
% hold on;
% plot_trajectory_with_orientation(ff, fusedTrajectory, step_size, vectorLength, 'Fused', 'm');
% hold off;
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% legend

figure()
hold on
plot(kalmanTrajectory.Point(1,:),kalmanTrajectory.Point(2,:))
plot(groundTruthTrajectory.Point(1,:),groundTruthTrajectory.Point(2,:))
plot(LiDARTrajectory.Point(1,:),LiDARTrajectory.Point(2,:))
plot(fusedTrajectory.Point(1,:),fusedTrajectory.Point(2,:))
% 
% figure()
% x = subplot(3,1,1);
% hold on
% plot(kalmanTrajectory.Time,kalmanTrajectory.Point(1,:))
% plot(groundTruthTrajectory.Time,groundTruthTrajectory.Point(1,:))
% plot(LiDARTrajectory.Time,LiDARTrajectory.Point(1,:))
% plot(fusedTrajectory.Time,fusedTrajectory.Point(1,:))
% 
% y = subplot(3,1,2);
% hold on
% plot(kalmanTrajectory.Time,kalmanTrajectory.Point(2,:))
% plot(groundTruthTrajectory.Time,groundTruthTrajectory.Point(2,:))
% plot(LiDARTrajectory.Time,LiDARTrajectory.Point(2,:))
% plot(fusedTrajectory.Time,fusedTrajectory.Point(2,:))
% 
% z = subplot(3,1,3);
% hold on
% plot(kalmanTrajectory.Time,kalmanTrajectory.Point(3,:))
% plot(groundTruthTrajectory.Time,groundTruthTrajectory.Point(3,:))
% plot(LiDARTrajectory.Time,LiDARTrajectory.Point(3,:))
% plot(fusedTrajectory.Time,fusedTrajectory.Point(3,:))
% 
% groundTruthTrajectory.Time = uint64(groundTruthTrajectory.Time * 1e9) + groundTruth_.time_start;
% kalmanTrajectory.Time = uint64(kalmanTrajectory.Time * 1e9) + groundTruth_.time_start;
% LiDARTrajectory.Time = uint64(LiDARTrajectory.Time * 1e9) + groundTruth_.time_start;
% fusedTrajectory.Time = uint64(fusedTrajectory.Time * 1e9) + groundTruth_.time_start;
% 
% linkaxes([x y z],'x')

%%

writeStructToTUMFormat(groundTruthTrajectory,"./traj_out/groundTruthTrajectory.txt");
writeStructToTUMFormat(kalmanTrajectory,"./traj_out/kalmanTrajectory.txt");
writeStructToTUMFormat(LiDARTrajectory,"./traj_out/LiDARTrajectory.txt");
writeStructToTUMFormat(fusedTrajectory,"./traj_out/fusedTrajectory.txt");
