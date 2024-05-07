function groundTruth_interp = interpolate_groundTruth(imuData, groundTruth_, sampleSize, slerp_bool)
    % interpolate_groundTruth - Interpolates ground truth data to match timestamps with IMU data.
    %
    % INPUTS:
    %   imuData - Struct containing IMU data with timestamps.
    %   groundTruth_ - Struct containing ground truth data with timestamps.
    %
    % OUTPUTS:
    %   groundTruth_interp - Interpolated ground truth data with aligned timestamps.

    groundTruth_interp = struct;
    groundTruth_interp.time_d = imuData.time_d;

    %% Initialize and Interpolate ground truth data for IMU
    if sampleSize == 0
        numSamples = length(imuData.time_d);
    else
        numSamples = sampleSize;
    end

    % Preallocate arrays for interpolated data
    groundTruth_interp.quat = quaternion.zeros(numSamples, 1);
    groundTruth_interp.pos = interp1(groundTruth_.time_d, groundTruth_.pos, imuData.time_d);

    % Iterate through each IMU data timestamp
    if slerp_bool
        for i = 1:numSamples
            % Find the closest timestamps before and after the current sensor timestamp
            [~, beforeIdx] = max(groundTruth_.time_d(groundTruth_.time_d <= imuData.time_d(i)));
            afterIdx = find(groundTruth_.time_d >= imuData.time_d(i), 1);
    
            if isempty(beforeIdx) || isempty(afterIdx) || afterIdx == beforeIdx
                % Handle edge cases: use the closest available quaternion
                closestIdx = find(min(abs(groundTruth_.time_d - imuData.time_d(i))) == abs(groundTruth_.time_d - imuData.time_d(i)), 1);
                groundTruth_interp.quat(i) = groundTruth_.quat(closestIdx);
            else
                % Interpolate quaternions using vectorized slerp
                t = (imuData.time_d(i) - groundTruth_.time_d(beforeIdx)) / (groundTruth_.time_d(afterIdx) - groundTruth_.time_d(beforeIdx));
                q_before = quaternion(groundTruth_.quat(beforeIdx, :));
                q_after = quaternion(groundTruth_.quat(afterIdx, :));
                groundTruth_interp.quat(i) = quatnormalize(slerp(q_before, q_after, t));
            end
        end
    end
    [W,X,Y,Z] = parts(groundTruth_interp.quat);
    groundTruth_interp.WXYZ = [W,X,Y,Z];
    
end
