% Close all open figures
close all;

playback_speed = 30;

% Extract and squeeze data from the structure, then downsample
time = squeeze(out.tout(1:playback_speed:end));
pose_base = squeeze(out.Q.Data(1:playback_speed:end,1:3));
poseTable = squeeze(out.poseTable.Data(1:playback_speed:end,1:3));
q_right = squeeze(out.Q.Data(1:playback_speed:end,4:10));
q_left = squeeze(out.Q.Data(1:playback_speed:end,11:17));
tau_ext = squeeze(out.tau_ext.Data(:,1,1:playback_speed:end));
trajRef = squeeze(out.trajRef.Data(:,1:playback_speed:end))';
X_rel = squeeze(out.X_rel.Data(:,1:playback_speed:end))';
F_ext = squeeze(out.F_ext.Data(1:playback_speed:end,:))';
rho_z = squeeze(out.rho_z.Data(1,:,1:playback_speed:end));
T_f = squeeze(out.T_f.Data(1:playback_speed:end));
T_i = squeeze(out.T_i.Data(1:playback_speed:end));

data_size = size(pose_base,1);
position_end_effector = zeros(data_size,6);

% Create a new figure and maximize it for better visualization
figure;
set(gcf, 'WindowState', 'maximized');

% Calculate end effector position errors without using absolute values
error_right_x = trajRef(:,1) - X_rel(:,1);
error_right_y = trajRef(:,2) - X_rel(:,2);
error_left_x = trajRef(:,4) - X_rel(:,4);
error_left_y = trajRef(:,5) - X_rel(:,5);

% Compute magnitude of tracking errors
error_right_mag = sqrt(error_right_x.^2 + error_right_y.^2);
error_left_mag = sqrt(error_left_x.^2 + error_left_y.^2);

% ---------------------------- Subplot 1: Tracking Error Magnitude ----------------------------
subplot(4, 1, 1);
plot(time, error_right_mag, 'Color', [0.8, 0.4, 0], 'LineWidth', 2.5); hold on;
plot(time, error_left_mag, 'Color', [0, 0.6, 0.2], 'LineWidth', 2.5);

% Title and Labels
title('Tracking Error Magnitude for Right and Left Arms', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Error Magnitude (m)', 'Interpreter', 'latex');

% Corrected legend
legend('Right Arm Tracking Error', 'Left Arm Tracking Error', ...
    'Interpreter', 'latex', 'Location', 'best');

% Customize subplot appearance
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); 

% ---------------------------- Subplot 2: Interaction Forces ----------------------------
subplot(4, 1, 2);
plot(time, F_ext(3,:), 'Color', [0.8, 0.4, 0], 'LineWidth', 2.5); hold on;
plot(time, F_ext(9,:), 'Color', [0, 0.6, 0.2], 'LineWidth', 2.5); 

% Title and Labels
title('External Interaction Forces ($F_{z}^{ext}$) on Right and Left Arms', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Force (N)', 'Interpreter', 'latex');

% Corrected legend
legend('Right Arm $F_{z}^{ext}$', 'Left Arm $F_{z}^{ext}$', ...
    'Interpreter', 'latex', 'Location', 'best');

% Customize subplot appearance
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); 

% ---------------------------- Subplot 3: Energy Storage in Tanks ----------------------------
subplot(4, 1, 3);
plot(time, T_f(:), 'r-', 'LineWidth', 2.5); hold on;
plot(time, T_i(:), 'b-', 'LineWidth', 2.5);

% Title and Labels
title('Energy Stored in Virtual Tanks ($T_f$ and $T_i$)', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Energy (J)', 'Interpreter', 'latex');

% Corrected legend
legend('$T_f$: Force Tracking Energy', '$T_i$: Impedance Control Energy', ...
    'Interpreter', 'latex', 'Location', 'best');

% Customize subplot appearance
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k'); 

%---------------------------- Subplot 4: Rho_z (Commented) ----------------------------
subplot(4, 1, 4);
plot(time, rho_z(1,:), 'r-', 'LineWidth', 2.5); hold on; % Right Arm
plot(time, rho_z(2,:), 'b-', 'LineWidth', 2.5); % Left Arm

% Title and Labels
title('Impedance Modulation Parameter $\rho_z$ for Right and Left Arms', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('$\rho_z$ (unit)', 'Interpreter', 'latex');

% Corrected legend
legend('Right Arm $\rho_z$', 'Left Arm $\rho_z$', ...
    'Interpreter', 'latex', 'Location', 'best');

% Customize subplot appearance
set(gca, 'FontSize', 14, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'k', 'YColor', 'k');
