function writeStructToTUMFormat(dataStruct, filePath)
    % This function writes position and orientation data from the input struct to a text file in TUM format.
    % 
    % Inputs:
    %   - dataStruct: The struct containing fields Time, Point, and Quat.
    %   - filePath: The path to the output .txt file.

    % Check that the required fields exist
    requiredFields = {'Time', 'Point', 'Quat'};
    for i = 1:length(requiredFields)
        if ~isfield(dataStruct, requiredFields{i})
            error('Field \"%s\" is missing in the input struct.', requiredFields{i});
        end
    end

    % Extract data from the struct
    time = dataStruct.Time;       % 1xN array
    position = dataStruct.Point; % 3xN array
    orientation = dataStruct.Quat; % Nx1 quaternion array

    % Open the file for writing
    fid = fopen(filePath, 'w');
    if fid == -1
        error('Failed to open file: %s', filePath);
    end

    % Write data in TUM format
    % Format: timestamp tx ty tz qx qy qz qw
    try
        for i = 1:length(time)
            % Extract position (tx, ty, tz)
            tx = position(1, i);
            ty = position(2, i);
            tz = position(3, i);

            % Extract orientation (qx, qy, qz, qw)
            [qw, qx, qy, qz] = parts(orientation(i)); % Extract quaternion parts

            % Write to file
            fprintf(fid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', time(i), tx, ty, tz, qx, qy, qz, qw);
        end
    catch ME
        fclose(fid);
        rethrow(ME);
    end

    % Close the file
    fclose(fid);
end
