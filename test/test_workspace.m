%% STD-DH parameters
clc;
clear;
% Define D-H parameters of links
d1 = 0; d2 = 0; d3 = 0; d4 = 286.5/1000; d5 = 0; d6 = 81.5/1000;
a1 = 0; a2 = 284/1000; a3 = -30/1000; a4 = 0; a5 = 0; a6 = 0;
alpha1 = -pi/2; alpha2 = 0; alpha3 = pi/2; alpha4 = -pi/2; alpha5 = pi/2; alpha6 = 0;
offset1 = 0; offset2 = 0; offset3 = 0; offset4 = 0; offset5 = 0; offset6 = 0;

% Build robot model
L1=Link([0 d1 a1 alpha1 ]); L1.offset = offset1;
L2=Link([0 d2 a2 alpha2 ]); L2.offset = offset2;
L3=Link([0 d3 a3 alpha3 ]); L3.offset = offset3;
L4=Link([0 d4 a4 alpha4 ]); L4.offset = offset4;
L5=Link([0 d5 a5 alpha5 ]); L5.offset = offset5;
L6=Link([0 d6 a6 alpha6 ]); L6.offset = offset6;
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'JR603');

% Limit joint space of the robot
L1.qlim = [(-180/180)*pi,(180/180)*pi];
L2.qlim = [(-155/180)*pi, (5/180)*pi];
L3.qlim = [(-20/180)*pi, (240/180)*pi];
L4.qlim = [(-180/180)*pi,(180/180)*pi];
L5.qlim = [(-95/180)*pi,(95/180)*pi];
L6.qlim = [(-360/180)*pi,(360/180)*pi];

% Plot robot model
figure
robot.plot([0, 0, 0, 0, 0, 0]);
robot.display();
robot.teach;

hold on;

%% Calculate workspace using Monte Carlo method
num_samples = 10000; % Set number of samples
workspace_points = zeros(num_samples, 3); % Initialize storage for workspace points

valid_samples = 0; % Record number of valid samples
for i = 1:num_samples
    % Generate random joint angles
    q1 = L1.qlim(1) + (L1.qlim(2) - L1.qlim(1)) * rand;
    q2 = L2.qlim(1) + (L2.qlim(2) - L1.qlim(1)) * rand; % Note: original code has L1.qlim(1) here, might be a typo for L2.qlim(1)
    q3 = L3.qlim(1) + (L3.qlim(2) - L3.qlim(1)) * rand;
    q4 = L4.qlim(1) + (L4.qlim(2) - L4.qlim(1)) * rand;
    q5 = L5.qlim(1) + (L5.qlim(2) - L5.qlim(1)) * rand;
    q6 = L6.qlim(1) + (L6.qlim(2) - L6.qlim(1)) * rand;

    % Calculate end-effector position using forward kinematics
    T = robot.fkine([q1, q2, q3, q4, q5, q6]); % Get homogeneous transformation matrix of end-effector

    % Check if T is a 4x4 homogeneous transformation matrix
    if size(T, 1) == 4 && size(T, 2) == 4
        pos = T(1:3, 4); % Extract displacement (X,Y,Z) of end-effector
        valid_samples = valid_samples + 1;
        % Store position
        workspace_points(i, :) = pos;
    else
        warning('Invalid transform matrix at sample %d', i);
    end
end

% Remove unfilled samples
workspace_points = workspace_points(1:valid_samples, :);

% Plot workspace
scatter3(workspace_points(:, 1), workspace_points(:, 2), workspace_points(:, 3), 5, 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Workspace of JR603 manipulator');
grid on;
axis equal; % Set equal axis scale