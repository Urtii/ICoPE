function plot_trajectory_with_time(st, dispName, color)
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

% Check struct fields
if ~isfield(st, 'Point') || ~isfield(st, 'Quat')
    error('Input struct must contain fields ''Point'' and ''Quat''.');
end

% Ensure dimensions are consistent
if size(st.Point, 2) ~= length(st.Quat)
    error('Number of points and quaternions must be the same.');
end

% Select the figure
% figure(ff);

% Plot trajectory
plot3(st.Point(1, :), st.Point(2, :), st.Time, 'DisplayName', dispName, 'Color', color);
hold on;

% Label axes for clarity
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('3D Body Trajectory and Orientations');

% hold off;

end
