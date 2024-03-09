% Close all open figures
close all;

playback_speed = 100;

% Extract and squeeze data from the structure, then downsample
time = squeeze(out.tout(1:playback_speed:end));
pose_base = squeeze(out.Q.Data(1:playback_speed:end,1:3));
q_right = squeeze(out.Q.Data(1:playback_speed:end,4:10));
q_left = squeeze(out.Q.Data(1:playback_speed:end,11:17));
data_size = size(pose_base,1);
position_end_effector = zeros(data_size,6);

% Now, every iteration of the loop corresponds to the downsampled data
for i = 1:data_size
    plot3(pose_base(1:i,1), pose_base(1:i,2), zeros(i, 1) ...
        , ':k', 'LineWidth', 1.5);
    hold on;  % Keep hold on to hold all plots.
    scatter3(pose_base(i,1), pose_base(i,2), 0, 'filled', 'MarkerFaceColor', 'k');
    position_end_effector(i,:) = plotMobileManipulator(pose_base(i,:), q_right(i,:), q_left(i,:))';
    pause(0.001);  % Small pause to allow for plot updates
end

figure; % Create a new figure
set(gcf, 'WindowState', 'maximized'); % Adjust size for a comprehensive layout

% Calculate the base pose error in X direction
initial_base_x = pose_base(1,1); % Initial base X position
error_base_x = pose_base(:,1) - initial_base_x;
initial_base_theta = pose_base(1,3); % Initial base theta
error_base_theta = pose_base(:,3) - initial_base_theta;

% Calculate end effector position errors without using absolute values
error_right_x = position_end_effector(:,1) - position_end_effector(1,1);
error_right_y = position_end_effector(:,2) - position_end_effector(1,2);
error_right_z = position_end_effector(:,3) - position_end_effector(1,3);
error_left_x = position_end_effector(:,4) - position_end_effector(1,4);
error_left_y = position_end_effector(:,5) - position_end_effector(1,5);
error_left_z = position_end_effector(:,6) - position_end_effector(1,6);

% Left Arm End Effector Position Errors
subplot(2, 2, 1);
plot(time, error_left_x, 'Color', [0.8, 0.4, 0], 'LineWidth', 2.5); hold on;
plot(time, error_left_y, 'Color', [0, 0.6, 0.2], 'LineWidth', 2.5);
plot(time, error_left_z, 'Color', [0.2, 0.4, 0.8], 'LineWidth', 2.5);
% Mark maximum error
[maxErrorX, maxIdxX] = max(abs(error_left_x));
[maxErrorY, maxIdxY] = max(abs(error_left_y));
[maxErrorZ, maxIdxZ] = max(abs(error_left_z));
[maxError, comp] = max([maxErrorX, maxErrorY, maxErrorZ]);
if comp == 1, maxIdx = maxIdxX; elseif comp == 2, maxIdx = maxIdxY; else, maxIdx = maxIdxZ; end
plot(time(maxIdx), error_left_z(maxIdx), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
text(time(maxIdx), error_left_z(maxIdx), sprintf('Max Left Arm Error: %.2f', maxError), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
title('Left Arm End Effector Position Errors');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X Error', 'Y Error', 'Z Error', 'Location', 'best');
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); % Customize this subplot

% Right Arm End Effector Position Errors
subplot(2, 2, 2);
plot(time, error_right_x, 'Color', [0.8, 0.4, 0], 'LineWidth', 2.5); hold on;
plot(time, error_right_y, 'Color', [0, 0.6, 0.2], 'LineWidth', 2.5);
plot(time, error_right_z, 'Color', [0.2, 0.4, 0.8], 'LineWidth', 2.5);
% Mark maximum error
[maxErrorX, maxIdxX] = max(abs(error_right_x));
[maxErrorY, maxIdxY] = max(abs(error_right_y));
[maxErrorZ, maxIdxZ] = max(abs(error_right_z));
[maxError, comp] = max([maxErrorX, maxErrorY, maxErrorZ]);
if comp == 1, maxIdx = maxIdxX; elseif comp == 2, maxIdx = maxIdxY; else, maxIdx = maxIdxZ; end
plot(time(maxIdx), error_right_z(maxIdx), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
text(time(maxIdx), error_right_z(maxIdx), sprintf('Max Right Arm Error: %.2f', maxError), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
title('Right Arm End Effector Position Errors');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X Error', 'Y Error', 'Z Error', 'Location', 'best');
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); % Customize this subplot

% Base Pose X Error
subplot(2, 2, [3, 4]);
yyaxis left;
x_plot = plot(time, error_base_x, 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth', 2.5); hold on; % Blue color
[maxErrorX, maxIdxX] = max(abs(error_base_x));
plot(time(maxIdxX), error_base_x(maxIdxX), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % specify line style
text(time(maxIdxX), error_base_x(maxIdxX), sprintf('Max Base X Error: %.2f', maxErrorX), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
ylabel('X Error (m)');
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'YColor', 'k');

% Base Pose Theta Error
yyaxis right;
theta_plot = plot(time, error_base_theta, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 2.5);
[maxErrorTheta, maxIdxTheta] = max(abs(error_base_theta));
plot(time(maxIdxTheta), error_base_theta(maxIdxTheta), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % specify line style
text(time(maxIdxTheta), error_base_theta(maxIdxTheta), sprintf('Max Base Theta Error: %.2f', maxErrorTheta), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
ylabel('Theta Error (rad)');
title('Base Pose Error');
xlabel('Time (s)');
legend([x_plot, theta_plot], {'X Error', 'Theta Error'}, 'Location', 'best'); % adjust legend
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'YColor', 'k'); % Customize this subplot


