function downsampled_GPS = downsample_GPS(data, downsample_factor)
    % DOWNSAMPLE_STRUCT Downsamples the fields of a struct by a specified factor.
    % 
    % Inputs:
    %   - data: Struct containing the data to be downsampled.
    %   - downsample_factor: Integer factor by which to downsample the data.
    %
    % Output:
    %   - downsampled_GPS: Struct containing the downsampled data.

    % Initialize the downsampled struct
    downsampled_GPS = struct();

    % Downsample relevant fields
    downsampled_GPS.time = data.time(1:downsample_factor:end);
    downsampled_GPS.LLA = data.LLA(1:downsample_factor:end, :);
    downsampled_GPS.Cov = data.Cov(1:downsample_factor:end, :, :);
    downsampled_GPS.time_norm = data.time_norm(1:downsample_factor:end);
    downsampled_GPS.time_d = data.time_d(1:downsample_factor:end);
    downsampled_GPS.velocity = data.velocity(1:downsample_factor:end, :);

    % Copy unchanged fields
    downsampled_GPS.time_start = data.time_start;
    downsampled_GPS.time_end = data.time_end;

    % Display a message confirming the downsampling
    fprintf('Data downsampled by a factor of %d.\n', downsample_factor);
end
