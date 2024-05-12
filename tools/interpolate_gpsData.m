function gpsData_interp = interpolate_gpsData(imuData, gpsData)
    % interpolate_gpsData - Interpolates GPS data to match timestamps with IMU data.
    %
    % INPUTS:
    %   imuData - Struct containing IMU data with timestamps.
    %   gpsData - Struct containing GPS data with timestamps.
    %
    % OUTPUTS:
    %   gpsData_interp - Interpolated GPS data with aligned timestamps.

    numSamples = length(imuData.time_d);

    % Initialize output struct
    gpsData_interp.velocity = NaN(numSamples, 3);
    gpsData_interp.LLA = NaN(numSamples, 3);

    % Preallocate variables
    lastClosestIdx = 1;
    sum_mindiff = 0;

    % Iterate through each 5Hz GPS data point
    for i = 1:size(gpsData.LLA, 1)
        currentTime = gpsData.time_d(i, 1);

        % Find the index in imuData that is closest or equal to the current timestamp
        closestIdx = find(imuData.time_d >= currentTime, 1);

        if isempty(closestIdx)
            gpsData_interp.LLA(numSamples, :) = gpsData.LLA(i, :);
            break
        end

        % Update the last closest index for the next iteration
        lastClosestIdx = closestIdx;
        sum_mindiff = sum_mindiff + (imuData.time_d(closestIdx) - currentTime)^2;

        % Assign the data value to the closest timestamp in the LLA matrix
        gpsData_interp.LLA(closestIdx, :) = gpsData.LLA(i, :);
    end

    % Compute and return the RMSE of timestamp differences
    sqrt(sum_mindiff / size(gpsData.LLA, 1))
end
