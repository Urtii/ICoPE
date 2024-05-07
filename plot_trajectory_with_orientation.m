function plot_trajectory_with_orientation(ff, st, step, vectorLength, dispName, color)
%PLOT_TRAJECTORY_WITH_ORIENTATION Plots the trajectory and body orientations.
%
%   PLOT_TRAJECTORY_WITH_ORIENTATION(ff, st, step, vectorLength, color) plots the trajectory of 3D body positions 
%   and their orientations on a given figure. It plots all trajectory points
%   and body orientations at intervals defined by 'step'.
%
%   Inputs:
%   ff            - Figure handle where the plot will be displayed
%   st            - Struct containing the fields:
%                   st.Point: 3-by-N matrix of 3D body positions
%                   st.Quat: N-by-1 array of quaternions representing orientation
%   step          - Integer defining the spacing between orientation plots
%   vectorLength  - Length of body orientation vectors
%   dispName      - Name of path in legend
%   color         - Color of the trajectory
%
%   Outputs:
%   A plot displayed in the figure identified by 'ff'.

% Validate input step
if ~isnumeric(step) || step <= 0
    error('Step must be a positive integer.');
end

% Check struct fields
if ~isfield(st, 'Point') || ~isfield(st, 'Quat')
    error('Input struct must contain fields ''Point'' and ''Quat''.');
end

% Ensure dimensions are consistent
if size(st.Point, 2) ~= length(st.Quat)
    error('Number of points and quaternions must be the same.');
end

% Select the figure
figure(ff);

% Plot trajectory
plot3(st.Point(1, :), st.Point(2, :), st.Point(3, :), 'DisplayName', dispName, 'Color', color);
hold on;

% Label axes for clarity
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Body Trajectory and Orientations');

% Plot orientation vectors at specified steps
for i = 1:step:length(st.Quat)
    % Extract the current point and quaternion
    pt = st.Point(:, i);
    quat = st.Quat(i, :);

    % Calculate the rotation matrix from quaternion
    R = quat2rotm(quat);

    % Define body axes vectors in local frame (assuming unit vectors)
    x_body = R(:, 1) * vectorLength;
    y_body = R(:, 2) * vectorLength;
    z_body = R(:, 3) * vectorLength;

    % Plot vectors from the current point
    quiver3(pt(1), pt(2), pt(3), x_body(1), x_body(2), x_body(3), 'r', 'LineWidth', 2,'HandleVisibility','off');
    quiver3(pt(1), pt(2), pt(3), y_body(1), y_body(2), y_body(3), 'g', 'LineWidth', 2,'HandleVisibility','off');
    quiver3(pt(1), pt(2), pt(3), z_body(1), z_body(2), z_body(3), 'b', 'LineWidth', 2,'HandleVisibility','off');
end

% hold off;

end
