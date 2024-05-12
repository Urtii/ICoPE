%% Get groundTruth Data
groundTruth_ = getMulRan_groundTruth('riverside2/global_pose.csv');

%% Get IMU Data
% imuData = getMulRan_IMUdata('riverside1_odom_noisy_old.bag',groundTruth_.time_start,groundTruth_.time_end);
[imuData, gpsData] = getMulRan_sensor_csv_data('riverside2/xsens_imu.csv','riverside2/gps.csv',groundTruth_.time_start,groundTruth_.time_end);

%%  Initialize and Interpolate ground truth data for imu

numSamples = length(imuData.time_d);

groundTruth_.quat_interp = quaternion.zeros(numSamples, 1);

groundTruth_.pos_interp = interp1(groundTruth_.time_d, groundTruth_.pos, imuData.time_d);

for i = 1:numSamples
    % Find the closest timestamps before and after the current sensor timestamp
    [~, beforeIdx] = max(groundTruth_.time_d(groundTruth_.time_d <= imuData.time_d(i)));
    afterIdx = find(groundTruth_.time_d >= imuData.time_d(i), 1);
    
    if isempty(beforeIdx) || isempty(afterIdx) || afterIdx == beforeIdx
        % Handle edge cases: use the closest available quaternion
        closestIdx = find(min(abs(groundTruth_.time_d - imuData.time_d(i))) == abs(groundTruth_.time_d - imuData.time_d(i)), 1);
        groundTruth_.quat_interp(i) = groundTruth_.quat(closestIdx);
    else
        % Interpolate quaternions
        t = (imuData.time_d(i) - groundTruth_.time_d(beforeIdx)) / (groundTruth_.time_d(afterIdx) - groundTruth_.time_d(beforeIdx));
        groundTruth_.quat_interp(i) = slerp(quaternion(groundTruth_.quat(beforeIdx,:)), quaternion(groundTruth_.quat(afterIdx,:)), t);
    end
end


clear groundTruth_rot groundTruth_quat_interp groundTruth_pos groundTruth_pos_interp i t afterIdx beforeIdx;

%%
groundTruth = timetable(groundTruth_.quat_interp,...
                    groundTruth_.pos_interp,'VariableNames',{'Orientation','Position'}, 'SampleRate', 100);
% groundTruth.Time = posixtime(groundTruth.Time);
%%
gravity = [0; 0; 9.81];
imuData.acc_body_wog = imuData.acc_body;
% for i = 1:numSamples
%      imuData.acc_body_wog(i,:) = imuData.acc_body_wog(i,:) - rotateframe(conj(groundTruth_.quat_interp(i)), gravity.');
% %      imuData.acc_body_wog(i,:) = rotateframe(groundTruth_.quat_interp(i),imuData.acc_body_wog(i,:)) -  gravity.';
% end


% decimationFactor = 1;
% numSamples_decimated = floor(length(accelData_body) / decimationFactor);
% numSamples = floor(length(accelData_body) / decimationFactor) * decimationFactor;

% accelData_body = accelData_body(1:numSamples, :);
% gyroData_body = gyroData_body(1:numSamples, :);
% timeStamps_norm = timeStamps_norm(1:numSamples, :);
NaN_matrix = NaN(numSamples,3);%imuData.acc_body_wog
gpsData.aligned = NaN(numSamples,3);%imuData.acc_body_wog

% Variables to keep track of the search in the 100Hz timestamps
lastClosestIdx = 1;
sum_mindiff = 0;

% Iterate through each 5Hz data point
for i = 1:size(gpsData.LLA, 1)
    currentTime = gpsData.time_d(i, 1);
    minDiff = inf;  % Initialize with a large number
    closestIdx = lastClosestIdx;
    
    % Search for the closest timestamp in 100Hz data, starting from the last closest index found
    for j = lastClosestIdx:length(imuData.time_d)
        tdiff = abs(imuData.time_d(j) - currentTime);
        if tdiff < minDiff
            minDiff = tdiff;
            closestIdx = j;
        elseif tdiff > minDiff
            % Since the timestamps are ordered and the difference started to increase,
            % the previous index was the closest
            break;
        end
    end
    
    % Update the last closest index for the next iteration
    lastClosestIdx = closestIdx;
    sum_mindiff = sum_mindiff + minDiff*minDiff;
    
    % Assign the data value to the closest timestamp in the aligned matrix
    gpsData.aligned(closestIdx, :) = gpsData.LLA(i, :);
end

sqrt(sum_mindiff / size(gpsData.LLA, 1))


% Now alignedData contains the closest samples, with NaNs elsewhere

sensorData = timetable(imuData.acc_body_wog, imuData.gyro_body,imuData.mag_body,gpsData.aligned,NaN_matrix, ...
                   'VariableNames', {'Accelerometer','Gyroscope','Magnetometer','GPSPosition','GPSVelocity'}, 'SampleRate', 100);
% sensorData.Time = posixtime(sensorData.Time);
          
%%

exfilter = load('insfilterAsyncTuneData.mat');
pretunedfilter = load('tunedAsyncFilter.mat');

imuFilter = insfilterAsync('ReferenceFrame', 'ENU', ...
    'ReferenceLocation', gpsData.LLA(1,:));

% imuFilter = pretunedfilter.imuFilter;

config = tunerconfig('insfilterAsync');
config.TunableParameters = setdiff(config.TunableParameters, ...
    {'GPSPositionNoise', 'GeomagneticVectorNoise', 'MagnetometerBiasNoise'...
     'GPSPositionNoise', 'GPSVelocityNoise', 'MagnetometerNoise'...
     'AccelerometerBiasNoise', 'GyroscopeBiasNoise'});
%     {'GPSVelocityNoise'});
config.MaxIterations = 1;
config.StepForward = 2;
 
measNoise = tunernoise('insfilterAsync');
measNoise.AccelerometerNoise = imuData.acc_Cov(1);
measNoise.GyroscopeNoise = imuData.gyro_Cov(1);
measNoise.GPSPositionNoise = mean(gpsData.Cov,'all');
% measNoise.AccelerometerNoise = 0.1;
% measNoise.GyroscopeNoise = 0.1;

init_state_quat = groundTruth_.quat_interp(1);
init_state_pos = groundTruth_.pos_interp(1,:);
[W,X,Y,Z] = parts(init_state_quat);
imuFilter.State(1:4) = [W;X;Y;Z];
imuFilter.State(8:10) = init_state_pos;
imuFilter.StateCovariance(1:4,1:4) = eye(4)*imuData.acc_Cov(1);
imuFilter.StateCovariance(8:10,8:10) = eye(3)*imuData.gyro_Cov(1);

init_state_vel = groundTruth_.pos(2,:).'/(groundTruth_.time_d(2)-groundTruth_.time_d(1));
imuFilter.State(11:13) = init_state_vel;

def_state = imuFilter.State;

%%

% tuned_params = tune(imuFilter,measNoise,sensorData,groundTruth,config);
% save('tunedAsyncFilter3.mat', 'imuFilter');
tuned_params = measNoise;

%%

% imuFilter.State = def_state;

dt =(diff(imuData.time_d));
numSamples = size(sensorData,1);
qEst = quaternion.zeros(numSamples,1);
posEst = zeros(numSamples,3);
% Iterate the filter for prediction and correction using sensor data.

% Accelerometer= imuData.acc_body;
Accelerometer_wog= imuData.acc_body_wog;%[imuData.acc_body(:,1) zeros(numSamples,2)];
Gyroscope= imuData.gyro_body;

for ii=1:numSamples
    if ii ~= 1
        predict(imuFilter, dt(ii-1));
    end
    if all(~isnan(Accelerometer_wog(ii,:)))
        fuseaccel(imuFilter,Accelerometer_wog(ii,:), ...
            tuned_params.AccelerometerNoise);
    end
    if all(~isnan(Gyroscope(ii,:)))
        fusegyro(imuFilter, Gyroscope(ii,:), ...
            tuned_params.GyroscopeNoise);
    end
    fusemag(imuFilter,imuData.mag_body(ii,:), ...
            tuned_params.MagnetometerNoise);
    if all(~isnan(gpsData.aligned(ii,:)))
        fusegps(imuFilter,gpsData.aligned(ii,:),tuned_params.MagnetometerNoise);
    end

    [posEst(ii,:), qEst(ii,:)] = pose(imuFilter);
end

%%
orientationError = rad2deg(dist(qEst, groundTruth.Orientation));
rmsorientationError = sqrt(mean(orientationError.^2))

positionError = sqrt(sum((posEst - groundTruth.Position).^2, 2));
rmspositionError = sqrt(mean( positionError.^2))

figure();
t = (0:numSamples-1)./ groundTruth.Properties.SampleRate;
subplot(2,1,1)
plot(t, positionError, 'b');
title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
xlabel('Time (s)');
ylabel('Position Error (meters)')
subplot(2,1,2)
plot(t, orientationError, 'b');
title("Orientation Error")
xlabel('Time (s)');
ylabel('Orientation Error (degrees)');

%%
figure();
grid on
plot3(posEst(:,1),posEst(:,2),posEst(:,3), 'b');
hold on
plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
plot3((gpsData.LLA(:,2)-gpsData.LLA(1,2))*100000,(gpsData.LLA(:,1)-gpsData.LLA(1,1))*100000,(gpsData.LLA(:,3)-gpsData.LLA(1,3)));
title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
xlabel('X');
ylabel('Y');
zlabel('Z');

%

% % positions = posEst;
% % orientations = qEst; % Nx4 matrix of quaternions [w, x, y, z]
% orientations = groundTruth_.quat_interp;
% positions = groundTruth_.pos_interp;
% quat = orientations(:);
% 
% % Number of positions/orientations
% n = size(positions, 1);
% 
% % Predefined length for the axes to draw
% axisLength = 50;
% 
% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% % Axis colors according to RGB convention
% axisColors = ['r', 'g', 'b']; % Red for X, Green for Y, Blue for Z
% 
% % Plot each position, orientation, and acceleration direction
% for i = 1:100:n
%     % Current position, orientation, and acceleration
%     pos = positions(i, :);
%     quat = orientations(i, :);
%     accDirBody = Accelerometer(i, :) / norm(Accelerometer(i, :)); % Normalize for direction
%     accDirBody_wog = Accelerometer_wog(i, :) / norm(Accelerometer_wog(i, :)); % Normalize for direction
%     
%     % Convert quaternion to rotation matrix
%     R = quat2rotm(quat); % Use the quat2rotm function provided earlier if needed
%     
%     % Manually compute the conjugate of the quaternion
%     quatConjugate = conj(quat);
%     
%     % Rotate acceleration direction to the world frame
%     accDirWorld = rotateframe(quatConjugate, accDirBody);
%     
%     % Define base axes
%     axes = eye(3) * axisLength;
%     axes(9) = 10 * axes(9);
%     
%     % Transform base axes by rotation matrix
%     transformedAxes = (R * axes')';
%     
%     % Plot position
%     plot3(pos(1), pos(2), pos(3), 'ko');
%     
%     % Draw orientation axes with RGB colors
%     for j = 1:3
%         endpt = pos + transformedAxes(j, :);
%         plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors(j), 'LineWidth', 2);
%     end
%     
%     % Draw acceleration direction in black in the world frame
%     accEndpt = pos + axisLength * accDirWorld'*norm(Accelerometer(i, :));
%     plot3([pos(1), accEndpt(1)], [pos(2), accEndpt(2)], [pos(3), accEndpt(3)], 'k', 'LineWidth', 2);
%     accEndpt_wog = pos + axisLength * accDirWorld'*norm(Accelerometer_wog(i, :));
%     plot3([pos(1), accEndpt_wog(1)], [pos(2), accEndpt_wog(2)], [pos(3), accEndpt_wog(3)], 'm', 'LineWidth', 2);
% end





% % Example data (replace with your data)
% % positions = groundTruth_.SE3(1:3,4,:); % Nx3 matrix of positions
% % positions = groundTruth_.pos;
% % orientations = groundTruth_.quat; % Nx4 matrix of quaternions [w, x, y, z]
% positions = posEst;
% orientations = qEst; % Nx4 matrix of quaternions [w, x, y, z]
% % ormat = groundTruth_.SE3(1:3,1:3,:);
% 
% % Number of positions/orientations
% n = size(positions, 1);
% 
% % Predefined length for the axes to draw
% axisLength = 50;
% 
% figure;
% hold on;
% grid on;
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% % Axis colors according to RGB convention
% axisColors = ['r', 'g', 'b']; % Red for X, Green for Y, Blue for Z
% 
% % Plot each position and orientation
% for i = 1:100:1000
%     % Current position and orientation
%     pos = positions(i, :);
%     quat = orientations(i, :);
%     
%     % Convert quaternion to rotation matrix
%     R = quat2rotm(quat); % Use the quat2rotm function provided earlier if needed
% %     R = ormat(:,:,i);
%     
%     % Define base axes
%     axes = eye(3) * axisLength;
%     axes(9) = 10*axes(9);
%     
%     % Transform base axes by rotation matrix
%     transformedAxes = (R * axes')';
%     
%     % Plot position
%     plot3(pos(1), pos(2), pos(3), 'ko');
%     
%     % Draw orientation axes with RGB colors
%     for j = 1:3
%         endpt = pos + transformedAxes(j, :);
%         plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors(j), 'LineWidth', 2);
%     end
% end
