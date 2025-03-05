% Close all open figures
close all;

playback_speed = 400;

% Extract and squeeze data from the structure, then downsample
time = squeeze(out.tout(1:playback_speed:end));
pose_base = squeeze(out.Q.Data(1:playback_speed:end,1:3));
poseTable = squeeze(out.poseTable.Data(1:playback_speed:end,1:3));
q_right = squeeze(out.Q.Data(1:playback_speed:end,4:10));
q_left = squeeze(out.Q.Data(1:playback_speed:end,11:17));
tau_ext = squeeze(out.tau_ext.Data(:,1,1:playback_speed:end));
trajRef = squeeze(out.trajRef.Data(:,1,1:playback_speed:end));
data_size = size(pose_base,1);
position_end_effector = zeros(data_size,6);

% Now, every iteration of the loop corresponds to the downsampled data
for i = 1:data_size
    plot3(pose_base(1:i,1), pose_base(1:i,2), zeros(i, 1) ...
        , ':k', 'LineWidth', 1.5);  % plot base path
    hold on;                            % Keep hold on to hold all plots.
    scatter3(pose_base(i,1), pose_base(i,2), 0, 'filled', 'MarkerFaceColor', 'k');
    position_end_effector(i,:) = plotMobileManipulator(pose_base(i,:), q_right(i,:), q_left(i,:), poseTable(i,:))';
    plot3(position_end_effector(1:i,1), position_end_effector(1:i,2), position_end_effector(1:i,3) ...
        , ':k', 'LineWidth', 1.5);      % plot right arm path
    plot3(position_end_effector(1:i,4), position_end_effector(1:i,5), position_end_effector(1:i,6) ...
        , ':k', 'LineWidth', 1.5);      % plot left arm path
    hold off;
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

% Base Pose X Error
subplot(4, 1, 1);
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

% Right Arm End Effector Position Errors
subplot(4, 1, 2);
plot(time, error_right_x, 'Color', [0.8, 0.4, 0], 'LineWidth', 2.5); hold on;
plot(time, error_right_y, 'Color', [0, 0.6, 0.2], 'LineWidth', 2.5);
plot(time, error_right_z, 'Color', [0.2, 0.4, 0.8], 'LineWidth', 2.5);
% Mark maximum error
[maxErrorX, maxIdxX] = max(abs(error_right_x));
[maxErrorY, maxIdxY] = max(abs(error_right_y));
[maxErrorZ, maxIdxZ] = max(abs(error_right_z));
[maxError, comp] = max([maxErrorX, maxErrorY, maxErrorZ]);
if comp == 1, maxIdx = maxIdxX; elseif comp == 2, maxIdx = maxIdxY; else, maxIdx = maxIdxZ; end
plot(time(maxIdx), -maxError, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
text(time(maxIdx), -maxError, sprintf('Max Right Arm Error: %.2f', maxError), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
title('Right Arm End Effector Position Errors');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X Error', 'Y Error', 'Z Error', 'Location', 'best');
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); % Customize this subplot

% Left Arm End Effector Position Errors
subplot(4, 1, 3);
plot(time, error_left_x, 'Color', [0.8, 0.4, 0], 'LineWidth', 2.5); hold on;
plot(time, error_left_y, 'Color', [0, 0.6, 0.2], 'LineWidth', 2.5);
plot(time, error_left_z, 'Color', [0.2, 0.4, 0.8], 'LineWidth', 2.5);
% Mark maximum error
[maxErrorX, maxIdxX] = max(abs(error_left_x));
[maxErrorY, maxIdxY] = max(abs(error_left_y));
[maxErrorZ, maxIdxZ] = max(abs(error_left_z));
[maxError, comp] = max([maxErrorX, maxErrorY, maxErrorZ]);
if comp == 1, maxIdx = maxIdxX; elseif comp == 2, maxIdx = maxIdxY; else, maxIdx = maxIdxZ; end
plot(time(maxIdx), -maxError, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
text(time(maxIdx), -maxError, sprintf('Max Left Arm Error: %.2f', maxError), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
title('Left Arm End Effector Position Errors');
xlabel('Time (s)');
ylabel('Error (m)');
legend('X Error', 'Y Error', 'Z Error', 'Location', 'best');
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); % Customize this subplot

% Disturbance Input Plot
subplot(4, 1, 4);
disturbance_x = tau_ext(1,:);
disturbance_theta = tau_ext(3,:);

yyaxis left; % Set up the left y-axis for position disturbance
plot(time, disturbance_x, 'Color', [0.6, 0.3, 0.1], 'LineWidth', 2.5);
ylabel('Pos Dist (N)');
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); % Customize left y-axis

yyaxis right; % Set up the right y-axis for orientation disturbance
plot(time, disturbance_theta, 'Color', [0.3, 0.6, 0.1], 'LineWidth', 2.5);
ylabel('Ori Dist (N.m)');
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); % Customize right y-axis

% Calculate y-axis limits
disturbance_x_min = min(disturbance_x);
disturbance_x_max = max(disturbance_x);
disturbance_theta_min = min(disturbance_theta);
disturbance_theta_max = max(disturbance_theta);

% Set y-axis limits with some buffer
y_axis_buffer = 0.1; % Adjust this value as needed
ylim_left = [disturbance_x_min - y_axis_buffer * abs(disturbance_x_min), disturbance_x_max + y_axis_buffer * abs(disturbance_x_max)];
ylim_right = [disturbance_theta_min - y_axis_buffer * abs(disturbance_theta_min), disturbance_theta_max + y_axis_buffer * abs(disturbance_theta_max)];

yyaxis left;
ylim(ylim_left);

yyaxis right;
ylim(ylim_right);

% Add legends to the plots
legend('Position Disturbance', 'Orientation Disturbance', 'Location', 'best');

xlabel('Time (s)');
title('Disturbance Input');



