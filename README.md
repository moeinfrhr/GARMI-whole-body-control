
# GARMI whole body controller in MATLAB

Whole body torque controller for a diff derive mobile service robot called GARMI that includes base as well as arms. 

### How to Run Simulation

1. Run `garmi_whole_body_control` script to start the whole body control simulation.
2. Execute `animate-simulation` to visualize the simulation.

### Adjusting Task Space Impedance Behavior

To modify the task space impedance behavior, update the following parameters in the `wholeBodyController` Matlab function:

```matlab
% Stiffness parameters
K_task_translation = 30; % Stiffness for translation components
K_task_orientation = 10; % Stiffness for orientation components

% Damping parameters
D_task_translation = 10; % Damping for translation components
D_task_orientation = 4; % Damping for orientation components

% Create diagonal matrices for task stiffness and damping
% Including both translation and orientation components
K_task = diag([repmat(K_task_translation, 1, 3), repmat(K_task_orientation, 1, 3)]);
D_task = diag([repmat(D_task_translation, 1, 3), repmat(D_task_orientation, 1, 3)]);

### Adjusting Null Space Behavior

To adjust the null space behavior, change the parameters within the wholeBodyController Matlab function as shown below:

% Base gains
K_null_base = 10; % Gain for the base
D_null_base = 45; % Damping for the base

% Right arm gains
K_null_right_arm = 1; % Example value, adjust as needed
D_null_right_arm = 1; % Example value, adjust as needed

% Left arm gains
K_null_left_arm = 1; % Example value, adjust as needed (can be different from right arm if desired)
D_null_left_arm = 1; % Example value, adjust as needed (can be different from right arm if desired)
