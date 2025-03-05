function position_end_effector = plotMobileManipulator(pose, q_right_arm, q_left_arm, poseTable)

% qhome_right = [0.942613,-1.37299,-0.0288793,-1.84589,0.131607,1.92806,-0.40];
% qhome_left = [-0.942613,-1.37299,-0.0288793,-1.84589,-0.131607,1.92806,0.40];

% Extract pose components
x = pose(1);
y = pose(2);
theta = pose(3);

% --- Parameters Definition ---
% Base dimensions
width = 0.75; % X-direction (width)
height = 0.2; % Y-direction (height)
depth = 0.75; % Z-direction (depth)

% Wheel parameters
wheelRadius = height / 2; % Radius of the wheels
wheelWidth = wheelRadius / 10; % Width of the wheels
axleHoleRadius = wheelRadius / 8; % Radius of the axle holes

% Spine length along the Z-axis and offset along the Y-axis
spineHeight = 1.05;
spineOffset = 0.1;

% Shoulder width along the X-axis
shoulderWidth = 0.4;

% Table parameters
widthTable = 0.4;
lengthTable = 0.8;

% --- Color Settings ---
wheelColor = [1, 0.75, 0]; % Orange
cuboidColor = [0, 0, 0]; % Black for edges
vertexColor = [1, 0, 0]; % Red
armColorRight = [0, 0, 0.8]; % Blue
armColorLeft = [0, 0.8, 0]; % Green

% compute poses in local frame:
% Corrected vertices definition with position adjustments
vertices = [-width/2 -depth/2 0; width/2 -depth/2 0; width/2 depth/2 0; -width/2 depth/2 0; % Bottom face vertices
    -width/2 -depth/2 height; width/2 -depth/2 height; width/2 depth/2 height; -width/2 depth/2 height]; % Top face vertices

% Edges connecting the vertices
edges = [1 2; 2 3; 3 4; 4 1; % Bottom face
    5 6; 6 7; 7 8; 8 5; % Top face
    1 5; 2 6; 3 7; 4 8]; % Connecting sides

% Wheel center positions in the local frame
wheelCenters = [ 0.0, depth/2+wheelWidth, height/6; % Front wheel center
    0.0, -depth/2-wheelWidth, height/6]; % Back wheel center

% Spine line in local frame
spineLineStart = [spineOffset, 0.0 , height];
spineLineEnd = [spineOffset, 0.0 , height + spineHeight];
spinePoints = [spineLineStart;spineLineEnd];

rightShoulderPose = [spineOffset , -shoulderWidth/2, height+ spineHeight]; % shoulderWidth/2 m to the right
leftShoulderPose = [spineOffset  , shoulderWidth/2, height+ spineHeight]; % shoulderWidth/2 m to the left
shoulderPoints = [rightShoulderPose;leftShoulderPose];

% plotting the global frame
origin_pos = [-0.7,0,0]; % Origin frame position XYZ- fixed frame
axisLength = 0.2; % Length of each axis
plotXYZCoordinate(origin_pos,0,'A', axisLength);
scatter3(origin_pos(1),origin_pos(2),origin_pos(3), 'filled', 'MarkerFaceColor', 'k');

% plotting the local frame
plotXYZCoordinate([x,y,0],theta,'B', axisLength);  % ploting the local frame

% ploting the table
plotTable(poseTable,lengthTable,widthTable,1.0);
poseTableFixed = [1.20;0.0;1.0];
plotTable(poseTableFixed,lengthTable,widthTable,0.1);

% Plot the base with rotation and translation applied
plotBase(x, y, theta, vertices, edges, cuboidColor);

%make auxilary arrows for kinematic representation at inital posture. 
quiver3(origin_pos(1),origin_pos(2),origin_pos(3), x-origin_pos(1),y-origin_pos(2),-origin_pos(3), 'Color', [.8, 0.5, 0.5], 'LineWidth', 2, 'MaxHeadSize', 0.4, 'AutoScale', 'off');
text(-0.35, 0.0, 0.0, '${}^A r_{p}$', 'FontSize', 16, 'Color', [.8, 0.5, 0.5], 'Interpreter', 'latex');
quiver3(x, y, 0, 0.43745, 0.2522, 1.1055, 'Color', [.8, 0.5, 0.5], 'LineWidth', 2, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
text(0.35, 0.0, 0.6, '${}^A r_{i/B}$', 'FontSize', 16, 'Color', [.8, 0.5, 0.5], 'Interpreter', 'latex');
quiver3(origin_pos(1),origin_pos(2),origin_pos(3), 0.43745-origin_pos(1),0.2522-origin_pos(2),1.1055-origin_pos(3), 'Color', [.8, 0.5, 0.5], 'LineWidth', 2, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
text(0.35, 0.0, 1, '${}^A r_{i/A}$', 'FontSize', 16, 'Color', [.8, 0.5, 0.5], 'Interpreter', 'latex');
text(0.433, -0.4, 1.25, '$i$', 'FontSize', 16, 'Color', [.8, 0.5, 0.5], 'Interpreter', 'latex');

% Calculate and plot wheels and axle holes with adjustments for pose
plotWheelsAndAxles(x, y, theta, wheelCenters, wheelRadius, axleHoleRadius,wheelColor);

% Calculate spine and shoulder positions, then plot
plotSpineAndShoulders(x, y, theta, spinePoints, shoulderPoints, vertexColor);

% Plot arms using DH parameters and joint angles
isRightArm = true;
position_end_effector(1:3) = plotArmChain(x, y, theta, isRightArm,q_right_arm, rightShoulderPose, armColorRight); % Green for Right arm
position_end_effector(4:6) = plotArmChain(x, y, theta,~isRightArm,q_left_arm, leftShoulderPose, armColorLeft); % Green for Left arm

% --- Plotting Commands ---
%view(3);
view(160,20)
% field_size = [-0.5 1.5 -1 2 -0.1 2];
% axis(field_size);
axis equal; % Locks the axis limits to the specified range
set(gcf, 'Color', 'w'); % Set background color to white
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;

end

% --- Auxiliary Functions ---
function [transformedPoint] = transformPoint(point, x, y, theta)
% Apply rotation
R = [cos(theta), -sin(theta), 0;
    sin(theta), cos(theta),  0;
    0,          0,           1];
% Apply translation
T = [x; y ; 0];
% Transform point
transformedPoint = R * point' + T;
end

function plotBase(x, y, theta, vertices, edges, color)

% Transform and plot each edge
for i = 1:size(edges, 1)
    edge = edges(i, :);
    v1 = transformPoint(vertices(edge(1), :), x, y, theta)';
    v2 = transformPoint(vertices(edge(2), :), x, y, theta)';
    plot3([v1(1), v2(1)], [v1(2), v2(2)], [v1(3), v2(3)], 'Color', color, 'LineWidth', 2);
end

% Transform and scatter plot each vertex
for i = 1:size(vertices, 1)
    transformedVertex = transformPoint(vertices(i, :), x, y, theta)';
    scatter3(transformedVertex(1), transformedVertex(2), transformedVertex(3), 'filled', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
end
end

function plotWheelsAndAxles(x, y, theta, wheelCenters, wheelRadius, axleHoleRadius, wheelColor)
angles = linspace(0, 2*pi, 100); % 100 points to form a smooth circle

for i = 1:size(wheelCenters,1)
    % Calculate the wheel and axle hole points after rotation
    xCircle = wheelRadius * cos(angles) + wheelCenters(i,1)*ones(size(angles));
    yCircle = wheelCenters(i,2)*ones(size(angles));
    zCircle = wheelRadius * sin(angles) + wheelCenters(i,3)*ones(size(angles));
    xHole = axleHoleRadius * cos(angles) + wheelCenters(i,1)*ones(size(angles));
    yHole = wheelCenters(i,2)*ones(size(angles));
    zHole = axleHoleRadius * sin(angles) + wheelCenters(i,3)*ones(size(angles));

    % Apply rotation and translation to the circle points
    for j = 1:length(angles)
        rotatedWheelPoint = transformPoint([xCircle(j), yCircle(j), zCircle(j)], x, y, theta)';
        rotatedAxleHolePoint = transformPoint([xHole(j), yHole(j), zHole(j)], x, y, theta)';

        % Replace the direct assignments with the rotated points
        xCircle(j) = rotatedWheelPoint(1);
        yCircle(j) = rotatedWheelPoint(2);
        zCircle(j) = rotatedWheelPoint(3);
        xHole(j) = rotatedAxleHolePoint(1);
        yHole(j) = rotatedAxleHolePoint(2);
        zHole(j) = rotatedAxleHolePoint(3);
    end

    % Plot wheel by filling between the rotated trajectories
    fill3(xCircle, yCircle, zCircle, wheelColor, 'LineStyle', 'none');
    plot3(xCircle, yCircle, zCircle, 'k', 'LineWidth', 2);

    % Plot axle holes
    fill3(xHole, yHole, zHole, 'k', 'LineWidth', 1);
end
end

function plotSpineAndShoulders(x, y, theta, spinePoints, shoulderPoints, vertexColor)

% Spine base point in global frame
spineBase = transformPoint(spinePoints(1,:), x, y, theta)';
spineTop = transformPoint(spinePoints(2,:), x, y, theta)';

% Plot spine
plot3([spineBase(1), spineTop(1)], [spineBase(2), spineTop(2)], [spineBase(3), spineTop(3)], 'k', 'LineWidth', 2);
scatter3([spineBase(1) spineTop(1)], [spineBase(2) spineTop(2)], [spineBase(3) spineTop(3)], 40, vertexColor, 'filled');

% Shoulders in global frame
rightShoulder = transformPoint(shoulderPoints(1,:), x, y, theta)';
leftShoulder = transformPoint(shoulderPoints(2,:), x, y, theta)';

% Plot shoulders
plot3([leftShoulder(1), rightShoulder(1)], [leftShoulder(2), rightShoulder(2)], [leftShoulder(3), rightShoulder(3)], 'k', 'LineWidth', 2);
scatter3([leftShoulder(1), rightShoulder(1)], [leftShoulder(2), rightShoulder(2)], [leftShoulder(3), rightShoulder(3)], 40, vertexColor, 'filled');

end

function position_end_effector = plotArmChain(x, y, theta, isRightArm, q, shoulderPose, armColor)
q_ext = [q,0];
joint_pos = zeros(length(q_ext) + 1, 3); % Preallocate memory for joint positions
joint_pos(1,:) = transformPoint(shoulderPose, x, y, theta)'; % Initialize first row with shoulder position in global frame

% --- DH parameters for 7DoF manipulator (Franka Emika, placeholder values) ---
l = 0.1;  % Tool length
d = [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107 + l]; % Last value includes tool length l if needed
a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0];
alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0];

% Rotation matrix to compensate the installation of Franka Arms on Garmi
% torso
if isRightArm
    beta = pi/2.0; % Rotation about Y axis
    gamma = pi/2.0; % Rotation about Z axis
else
    beta = pi/2.0; % Rotation about Y axis
    gamma = -pi/2.0; % Rotation about Z axis
end

Ty = [cos(beta), 0, sin(beta), 0;
    0, 1, 0, 0;
    -sin(beta), 0, cos(beta), 0;
    0, 0, 0, 1];
Tz = [cos(gamma), -sin(gamma), 0, 0;
    sin(gamma), cos(gamma), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
T = Ty * Tz; % arm rotation matrix to compensate for mounting installation.

% Compute and plot each joint position
for i = 1:length(q_ext)
    % Adjusting indices for MATLAB (1-based indexing)
    if i > length(q_ext) % Ensure we don't exceed the DH parameters array
        break;
    end

    % DH Transformation matrix for the current joint
    ct = cos(q_ext(i));
    st = sin(q_ext(i));
    ca = cos(alpha(i));
    sa = sin(alpha(i));

    Ti = [ct, -st, 0, a(i);
          st * ca, ct * ca, -sa, -sa * d(i);
          st * sa, ct * sa, ca, ca * d(i);
          0, 0, 0, 1];

    T = T * Ti; % Update transformation matrix
    % Joint position in global frame
    next_joint_pos = transformPoint(T(1:3, 4)' + shoulderPose, x, y, theta)' ;
    joint_pos(i+1,:) = next_joint_pos; % Assign next joint position
end
% Remove any preallocated but unused rows
joint_pos = joint_pos(1:i+1,:);

plot3(joint_pos(1:end-1,1), joint_pos(1:end-1,2), joint_pos(1:end-1,3), 'Color', armColor, 'LineWidth', 2);
scatter3(joint_pos(1:end-1,1), joint_pos(1:end-1,2), joint_pos(1:end-1,3), 'filled', 'MarkerEdgeColor', armColor, 'MarkerFaceColor', armColor);
plot3(joint_pos(end-1:end,1), joint_pos(end-1:end,2), joint_pos(end-1:end,3), 'Color', [.5,.5,0], 'LineWidth', 2);
scatter3(joint_pos(end,1), joint_pos(end,2), joint_pos(end,3), 'filled', 'MarkerEdgeColor', [.5,.5,0], 'MarkerFaceColor', [.5,.5,0]);
position_end_effector = joint_pos(end,:);
end

function plotXYZCoordinate(position, theta, frameName, axisLength)
% position: A 1x3 vector specifying the XYZ coordinates of the origin of the coordinate system to be plotted.
% theta: Rotation angle about the Z axis in radians.
% frameName: A character vector or string specifying the frame name.
% axisLength: Scalar value defining the length of each axis line.

% Rotation matrix for Z axis
Rz = [cos(theta) -sin(theta) 0;
      sin(theta) cos(theta)  0;
      0           0            1];

% Extract the position components
x = position(1);
y = position(2);
z = position(3);

% Original axis directions: For X, Y, and Z axes respectively
u = [axisLength, 0, 0]; % X axis direction
v = [0, axisLength, 0]; % Y axis direction
w = [0, 0, axisLength]; % Z axis direction

% Rotate X and Y axis directions
u_rotated = Rz * u.';
v_rotated = Rz * v.';

% Plot the X axis in red
quiver3(x, y, z, u_rotated(1), u_rotated(2), u_rotated(3), 'Color', [1,0,0], 'LineWidth', 2, 'MaxHeadSize', 1.5, 'AutoScale', 'off');

% Plot the Y axis in green
quiver3(x, y, z, v_rotated(1), v_rotated(2), v_rotated(3), 'Color', [0,1,0], 'LineWidth', 2, 'MaxHeadSize', 1.5, 'AutoScale', 'off');

% Plot the Z axis in blue (unchanged)
quiver3(x, y, z, w(1), w(2), w(3), 'Color', [0,0,1], 'LineWidth', 2, 'MaxHeadSize', 1.5, 'AutoScale', 'off');

% Dynamically generate axis labels based on frame name
x_label = strcat('$X_{', frameName, '}$');
y_label = strcat('$Y_{', frameName, '}$');
z_label = strcat('$Z_{', frameName, '}$');

% Label axes with LaTeX interpreter
text(x + u_rotated(1), y + u_rotated(2), z + u_rotated(3), x_label, 'FontSize', 12, 'Color', 'r', 'Interpreter', 'latex');
text(x + v_rotated(1), y + v_rotated(2), z + v_rotated(3), y_label, 'FontSize', 12, 'Color', 'g', 'Interpreter', 'latex');
text(x + w(1), y + w(2), z + w(3) + 0.07, z_label, 'FontSize', 12, 'Color', 'b', 'Interpreter', 'latex');

% --- Plotting Commands ---
xlabel(x_label, 'Interpreter', 'latex');
ylabel(y_label, 'Interpreter', 'latex');
zlabel(z_label, 'Interpreter', 'latex');
end

function plotTable(p, w, l, transparency)
% plotTable - Plots a rectangular table centered at a specified position in 3D space.
%
% Syntax: plotTable(p, w, l, transparency)
%
% Inputs:
%   p - A 1x3 vector specifying the XYZ coordinates of the center of the table.
%   w - The width of the table.
%   l - The length of the table.
%   transparency - A scalar between 0 and 1 specifying the table's transparency.
%
% Example:
%   plotTable([1, 2, 0], 4, 6, 0.5);

    % Extract the center position
    x = p(1);
    y = p(2);
    z = p(3);

    % Calculate the rectangle's corner points
    half_w = w / 2;
    half_l = l / 2;

    % Rectangle vertices (clockwise order)
    rectX = [x - half_l, x + half_l, x + half_l, x - half_l];
    rectY = [y - half_w, y - half_w, y + half_w, y + half_w];
    rectZ = z * ones(1, 4); % Table lies in the same Z-plane

    % Plot the filled table surface
    fill3(rectX, rectY, rectZ, 'c', 'FaceAlpha', transparency, 'EdgeColor', 'b', 'LineWidth', 2);
end










