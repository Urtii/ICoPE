%% Get groundTruth Data
groundTruth_ = getMulRan_groundTruth('riverside1/global_pose.csv');

%% Plot 3D Orientation

n = size(groundTruth_.time, 1);
axisLength = 20;
axisColors = ['r', 'g', 'b']; % Red for X, Green for Y, Blue for Z
% axisColors_acc = ['m', 'k', 'c']; % Magenta for X, Black for Y, Cyan for Z

figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
for i = 1:500:n

    R_gt = groundTruth_.SE3(1:3,1:3,i);
    pos = groundTruth_.pos(i,:);

    % Principal axes
    x_axis = R_gt(:,1) * axisLength;
    y_axis = R_gt(:,2) * axisLength;
    z_axis = R_gt(:,3) * axisLength;

    % Plot position
    plot3(pos(1), pos(2), pos(3), 'ko');

    % Draw orientation axes with RGB colors
    for j = 1:3
        % Plot each axis using quiver3
        quiver3(pos(1), pos(2), pos(3), x_axis(1), x_axis(2), x_axis(3),0, 'r', 'LineWidth', 2);
        quiver3(pos(1), pos(2), pos(3), y_axis(1), y_axis(2), y_axis(3),0, 'g', 'LineWidth', 2);
        quiver3(pos(1), pos(2), pos(3), z_axis(1), z_axis(2), z_axis(3),0, 'b', 'LineWidth', 2);
    end

end