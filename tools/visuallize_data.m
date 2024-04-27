%% Get groundTruth Data
groundTruth_ = getMulRan_groundTruth('riverside1/global_pose.csv',30000);

%% Get IMU Data
% [imuData, gpsData] = getMulRan_sensor_csv_data('riverside1/xsens_imu.csv','riverside1/gps.csv',groundTruth_.time_start,groundTruth_.time_end);

%% Get LiDAR Odometry Data
% odom = get_odom_from_bag("riverside1/clean_odom_tfix.bag",groundTruth_.time_start,groundTruth_.time_end);
% odom = get_odom_from_bag("riverside1/clean_odom_tfix.bag");

%% Plot Position

% % odom.pos = rotateframe(groundTruth_.quat(1), odom.pos);
% % odom.quat = groundTruth_.quat(1)*odom.quat;
% 
% figure();
% grid on
% % plot3(posEst(:,1),posEst(:,2),posEst(:,3), 'b');
% hold on
% plot3(groundTruth_.pos(:,1),groundTruth_.pos(:,2),groundTruth_.pos(:,3));
% plot3(odom.pos(:,1),odom.pos(:,2),odom.pos(:,3));
% % plot3((gpsData.LLA(:,2)-gpsData.LLA(1,2))*100000,(gpsData.LLA(:,1)-gpsData.LLA(1,1))*100000,(gpsData.LLA(:,3)-gpsData.LLA(1,3)));
% title("Tuned insfilterAsync" + newline + "Euclidean Distance Position Error")
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

%% Plot 3D Orientation

n = length(odom.time_d);
axisLength = 20;
axisColors = ['r', 'g', 'b']; % Red for X, Green for Y, Blue for Z
axisColors_acc = ['m', 'k', 'c']; % Magenta for X, Black for Y, Cyan for Z

figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

plot3(odom.pos(:,1),odom.pos(:,2),odom.pos(:,3));
for i = 1:100:n
    % Current position, orientation, and acceleration
%     pos = posEst(i, :);
%     quat_wb = qEst(i, :);
%     acc_vec = rotateframe(groundTruth_interp.quat(i), imuData.acc_body(i,:));
    pos = odom.pos(i, :);
%     quat_wb = groundTruth_interp.quat(i, :);
    
    % Convert quaternion to rotation matrix
    R_wb = quat2rotm(odom.quat(i)); % Use the quat2rotm function provided earlier if needed
    
    % Define base axes
    axes = eye(3) * axisLength;
%     R_acc = diag(acc_vec)*axisLength; % Use the quat2rotm function provided earlier if needed
%     R_acc(3,3) = R_acc(3,3)/10;
%     axes = diag(accDirBody_wog)*axisLength;
%     norm(accDirWorld_wog)
%     axes(9) = 10 * axes(9);
    
    % Transform base axes by rotation matrix
    transformedAxes = (R_wb * axes')';
    
    % Plot position
    plot3(pos(1), pos(2), pos(3), 'ko');
    
    % Draw orientation axes with RGB colors
    for j = 1:3
        endpt = pos + transformedAxes(j, :);
        plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors(j), 'LineWidth', 2);
%         endpt = pos + R_acc(j, :);
%         plot3([pos(1), endpt(1)], [pos(2), endpt(2)], [pos(3), endpt(3)], axisColors_acc(j), 'LineWidth', 2);
    end
    
end

%%  Initialize and Interpolate ground truth data
% 
% numSamples = length(imuData.time_d);
% 
% groundTruth_.quat_interp = quaternion.zeros(numSamples, 1);
% 
% groundTruth_.pos_interp = interp1(groundTruth_.time_d, groundTruth_.pos, imuData.time_d);
% 
% for i = 1:numSamples
%     % Find the closest timestamps before and after the current sensor timestamp
%     [~, beforeIdx] = max(groundTruth_.time_d(groundTruth_.time_d <= imuData.time_d(i)));
%     afterIdx = find(groundTruth_.time_d >= imuData.time_d(i), 1);
%     
%     if isempty(beforeIdx) || isempty(afterIdx) || afterIdx == beforeIdx
%         % Handle edge cases: use the closest available quaternion
%         closestIdx = find(min(abs(groundTruth_.time_d - imuData.time_d(i))) == abs(groundTruth_.time_d - imuData.time_d(i)), 1);
%         groundTruth_.quat_interp(i) = groundTruth_.quat(closestIdx);
%     else
%         % Interpolate quaternions
%         t = (imuData.time_d(i) - groundTruth_.time_d(beforeIdx)) / (groundTruth_.time_d(afterIdx) - groundTruth_.time_d(beforeIdx));
%         groundTruth_.quat_interp(i) = slerp(quaternion(groundTruth_.quat(beforeIdx,:)), quaternion(groundTruth_.quat(afterIdx,:)), t);
%     end
% end
% 
% 
% clear groundTruth_rot groundTruth_quat_interp groundTruth_pos groundTruth_pos_interp i t afterIdx beforeIdx;
% 
% %%
% groundTruth = timetable(groundTruth_.quat_interp,...
%                     groundTruth_.pos_interp,'VariableNames',{'Orientation','Position'}, 'SampleRate', 100);
% %%
% 
% imuData.acc_body = imuData.acc_body;
% 
% gravity = [0; 0; 9.81];
% imuData.acc_body_smooth = smoothdata(imuData.acc_body,1,"movmean",11);
% imuData.acc_body_wog = imuData.acc_body;
% for i = 1:numSamples
% %     imuData.acc_body_wog(i,:) = imuData.acc_body_wog(i,:) - gravity.';
%      imuData.acc_body_wog(i,:) = imuData.acc_body_smooth(i,:) - rotateframe(conj(groundTruth_.quat_interp(i)), gravity.');
% end
%           
% Accelerometer= imuData.acc_body_smooth;
% Accelerometer_wog= imuData.acc_body_wog;%[imuData.acc_body(:,1) zeros(numSamples,2)];
% 
% %%
% 
% % positions = posEst;
% % orientations = qEst; % Nx4 matrix of quaternions [w, x, y, z]
% orientations = groundTruth_.quat_interp;
% positions = groundTruth_.pos_interp;
% 
% % Number of positions/orientations
% n = size(positions, 1);
% 
% % Predefined length for the axes to draw
% axisLength = 20;
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
% accW = Accelerometer;
% accW_wog = Accelerometer;
% for i = 1:100:n
%     % Current position, orientation, and acceleration
%     pos = positions(i, :);
%     quat_wb = orientations(i, :);
%     accDirBody = Accelerometer(i, :) / norm(Accelerometer(i, :)); % Normalize for direction
%     accDirBody_wog = Accelerometer_wog(i, :) ;%/ norm(Accelerometer_wog(i, :)); % Normalize for direction
%     
%     % Convert quaternion to rotation matrix
%     R_wb = quat2rotm(quat_wb); % Use the quat2rotm function provided earlier if needed
%     
%     % Manually compute the conjugate of the quaternion
%     quat_bw = conj(quat_wb);
%     
%     % Rotate acceleration direction to the world frame
%     accDirWorld = rotateframe(quat_wb, accDirBody);
%     accDirWorld_wog = rotateframe(quat_wb, accDirBody_wog);
%     norm(accDirWorld)
%     norm(accDirWorld_wog)
%     
%     % Define base axes
% %     axes = eye(3) * axisLength;
%     axes = diag(accDirBody_wog)*axisLength;
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
%     end
%     
%     % Draw acceleration direction in black in the world frame
% %     accW(i,:) = axisLength * accDirWorld.';
% %     accEndpt = pos + accW(i,:);
% %     plot3([pos(1), accEndpt(1)], [pos(2), accEndpt(2)], [pos(3), accEndpt(3)], 'k', 'LineWidth', 2);
% %     accW_wog(i,:) = axisLength * accDirWorld_wog.';
% %     accEndpt_wog = pos + accW_wog(i,:);
% %     plot3([pos(1), accEndpt_wog(1)], [pos(2), accEndpt_wog(2)], [pos(3), accEndpt_wog(3)], 'm', 'LineWidth', 2);
%     
% end
% 
% figure
% subplot(3,1,1)
% hold on
% plot(imuData.acc_body(:,1))
% plot(imuData.acc_body_wog(:,1))
% plot(accW(:,1)/50)
% plot(accW_wog(:,1)/50)
% subplot(3,1,2)
% hold on
% plot(imuData.acc_body(:,2))
% plot(imuData.acc_body_wog(:,2))
% plot(accW(:,2)/50)
% plot(accW_wog(:,2)/50)
% subplot(3,1,3)
% hold on
% plot(imuData.acc_body(:,3))
% plot(imuData.acc_body_wog(:,3))
% plot(accW(:,3)/50)
% plot(accW_wog(:,3)/50)
% 
% %norm(Accelerometer_wog(i, :))
