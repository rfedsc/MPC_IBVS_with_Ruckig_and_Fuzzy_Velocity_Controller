% Clear workspace, command window and close all figures for clean execution
clear     
clc       
close all 
% Set numerical format to long precision for higher accuracy
format long g
% Add current directory and rucking_mex directory to MATLAB path for function access
addpath(genpath('.\'));
rucking_mex_dir = '../rucking_mex';
addpath(rucking_mex_dir);

%% MPC Parameter - Core parameters for Model Predictive Control
fx=800;
fy=800;
cx=800;
cy=800;
K = [fx,0,cx;0,fy,cy;0,0,1];  % Camera intrinsic matrix
image_width=1600;
image_height=1600
F = eye(8,8);    
Np = 6;          
Q = eye(8)*10;   R_weight=eye(6)*1;
Lambda=1;
%Q = diag([1, 1, 1, 1, 1, 1, 1, 1]); 
%Q = diag([8, 8, 8, 8, 8, 8, 8, 8]); 
%R_weight=0.2;   
R = eye(6);      
%R_weight=0.2;   
W_weight=8;      
Tf = 100;        
Ts = 0.15;       
t = 0:Ts:Tf;     
N = length(t);   
s_history = zeros(8,N); 
tau_history = zeros(6,N);
Z_history = zeros(1,N); 
vis_pose = true; 
vis_predict = false; 

% Create figure for predicted feature points history visualization
fig_handle = figure('Name', 'Predicted Feature Points History');
set(fig_handle, 'Position', [1500, 500, 600, 500]); 

%% Robot Model Initialization - JR603 6-DOF manipulator
figure(1)
title('JR603')
axis([-1 1 -1 1 -1 1]); 
% DH parameters for JR603 manipulator (unit: m/rad)
d = [0,0,0,286.5/1000,0,81.5/1000];     
a = [0,284/1000,-30/1000,0,0,0];        
alp = [-1.5708, 0, 1.5708, -1.5708, 1.5708, 0]; 
offset = [0,0,0,0,0,0]; 

% Create individual links using DH parameters
L1=Link([0          d(1)       a(1)       alp(1)   ]);L1.offset = offset(1);
L2=Link([0          d(2)       a(2)       alp(2)   ]);L2.offset = offset(2);
L3=Link([0          d(3)       a(3)       alp(3)   ]);L3.offset = offset(3);
L4=Link([0          d(4)       a(4)       alp(4)   ]);L4.offset = offset(4);
L5=Link([0          d(5)       a(5)       alp(5)   ]);L5.offset = offset(5);
L6=Link([0          d(6)       a(6)       alp(6)   ]);L6.offset = offset(6);
% Assemble serial link manipulator and name it JR603
JR603=SerialLink([L1 L2 L3 L4 L5 L6],'name','JR603');
% Initial joint angles (rad) - calibrated for task space reachability
%theta_initial = [0*pi/180,-90*pi/180,180*pi/180,0*pi/180,90*pi/180,0*pi/180];
theta_initial = [45*pi/180,-90*pi/180,180*pi/180,0*pi/180,90*pi/180,154*pi/180];
%theta_initial = [45*pi/180,-90*pi/180,180*pi/180,0*pi/180,90*pi/180,90*pi/180];
% Plot initial robot pose with customized workspace and view angle
JR603.plot(theta_initial, 'workspace', [-1 1 -1 1 -1 1], 'delay', 0, 'view', [135 35], 'noarrow', 'nowrist');
JR603.teach() % Enable interactive teaching mode for manipulator
hold on 

%% Color Palette - For data visualization (plots/curves)
colors = [
0.1216 0.4667 0.7059;  % Blue
1.0000 0.4980 0.0549;  % Orange
0.1725 0.6275 0.1725;  % Green
0.9 0 0 ;              % Red
0.5804 0.4039 0.7412;  % Purple
0.5490 0.3373 0.2941;  % Brown
0.8902 0.4667 0.7608;  % Pink
0.4980 0.4980 0.4980;  % Gray
]; 

colors_ = [
 1 0 0;                % Red
 0.1725 0.6275 0.1725; % Green
 1 0 1;                % Magenta
 0.850980392156863   0.325490196078431   0.098039215686275; % Orange-red
 0.850980392156863   0.325490196078431   0.098039215686275; % Orange-red
 0.074509803921569   0.623529411764706   1.000000000000000; % Sky blue
 ];

%% Task Space Visualization - Coordinate frame and grid
% Identity matrix for space frame transformation
TS = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1;];
hold on 
% Set axis limits for task space visualization
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
length_ = 0.5; % Grid size for task space plane (m)
height = 0;    % Height of task space plane (m)
% Generate grid coordinates for task space plane
x_length = -0.5:length_*2/10:length_;
y_length = -0.5:length_*2/10:length_;
[x_grid,y_grid] = meshgrid(x_length,y_length);
mesh(x_grid,y_grid,height.*ones(11,11)); % Plot 2D grid in 3D space

%% Camera calibration - Intrinsic/extrinsic parameters
% Camera intrinsic matrix (alternative definition)
a = [800    0  800   0;
       0  800  800   0;
       0    0    1   0;];
% Transformation matrix from end-effector to camera (eMc)
eMc = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0,0,0,1;];                

%% Desired Parameter - Visual servoing target
% Desired pixel coordinates of 4 feature points (u,v)
target_pixel = [ 600  600 1000 1000;
                 1000  600 1000  600;];
% Calculate center of desired feature points (for error evaluation)
target_center_x = mean(target_pixel(1,:));
target_center_y = mean(target_pixel(2,:));

% Create figure for camera image visualization
figure(2)
set(gcf, 'Color', 'w');
xlim([0 1600]) % Pixel range x-axis
ylim([0 1600]) % Pixel range y-axis
xlabel('$u$~(pixels)', 'Interpreter', 'latex', 'FontSize', 22, 'FontWeight', 'bold');
ylabel('$v$~(pixels)', 'Interpreter', 'latex', 'FontSize', 22, 'FontWeight', 'bold');
hold on
% Plot desired feature points (blue star) and connect with lines
plot(target_pixel(1,1),target_pixel(2,1),'b*')
plot(target_pixel(1,2),target_pixel(2,2),'b*')
plot(target_pixel(1,3),target_pixel(2,3),'b*')
plot(target_pixel(1,4),target_pixel(2,4),'b*')
line(target_pixel(1,[1 2]),target_pixel(2,[1 2]),'color','b');
line(target_pixel(1,[2 4]),target_pixel(2,[2 4]),'color','b');
line(target_pixel(1,[4 3]),target_pixel(2,[4 3]),'color','b');
line(target_pixel(1,[3 1]),target_pixel(2,[3 1]),'color','b');


disp(target_pixel);
s_target = Calculate_s(target_pixel,a);
target_pixels = Calculate_e(target_pixel);
z_star = 0.1; 

%% Control Parameter - Servoing and iteration settings
iteration_limit = 1000;     
error_max = 0.03;           
joint_velocity_max = 15;    
Hz = 25;                    
theta_current = theta_initial; % Initialize joint angles to initial pose
disp('theta_initial:');
disp(theta_initial);
% Calculate initial end-effector pose using forward kinematics
T_s_e_current = jr603_fkine(theta_current);

%% Data Storage Initialization
% Feature error history
x = []; y = []; i_e = [];
x1_e = []; y1_e = []; x2_e = []; y2_e = [];
x3_e = []; y3_e = []; x4_e = []; y4_e = [];
% Camera velocity history
x_v = []; y_v = []; z_v = []; rx_v = []; ry_v = []; rz_v = [];
% Joint velocity/angle history
joint1_velocity = []; joint2_velocity = []; joint3_velocity = [];
joint4_velocity = []; joint5_velocity = []; joint6_velocity = [];
joint1_angle = []; joint2_angle = []; joint3_angle = [];
joint4_angle = []; joint5_angle = []; joint6_angle = [];
% Fuzzy control parameters
fuzzy_lambda_values = []; a_lambda_values = [];
% Feature error initialization
e_current=[0 0 0 0 0 0 0 0]';
e_diff=0; e_last=0;

%% Trajectory Interpolation - 5th-order polynomial interpolation
traj_steps = 1;                    % Number of interpolation steps per control cycle
interp_time = Ts/traj_steps        % Time step for interpolation (s)
traj_points = zeros(traj_steps,6)   % Storage for interpolated joint angles
prev_joint_angles = theta_current;  % Previous joint angles for interpolation

%% Ruckig Trajectory Generator - Jerk-limited trajectory planning
j=0; step_time=0;
rucking_steps = 50;                % Ruckig update steps per control cycle
ruckig_init_pos = theta_initial;   % Initial position for Ruckig
ruckig_init_vel = zeros(1,6);      % Initial velocity (rad/s)
ruckig_init_acc = zeros(1,6);      % Initial acceleration (rad/s²)
ruckig_target_vel = zeros(1,6);    % Target velocity (rad/s)
ruckig_target_acc = zeros(1,6);    % Target acceleration (rad/s²)
rucking_max_acc = 10.0*ones(1,6);  % Max acceleration (rad/s²)
rucking_max_jerk = 50.0*ones(1,6); % Max jerk (rad/s³) - smooth motion constraint

a_rated = 15; % Rated acceleration for fuzzy controller
% Initialize Ruckig trajectory generator with constraints
ruckig_mex('initialize',...
    ruckig_init_pos,...
    ruckig_init_vel,...
    ruckig_init_acc,...
    ruckig_target_vel,...
    ruckig_target_acc,...
    rucking_max_acc,...
    rucking_max_jerk);

%% Error Accumulation - For fuzzy velocity controller
e_sum = zeros(6,1);    % Cumulative velocity error
jerk_sum = zeros(6,1); % Cumulative jerk
acc_sum = zeros(6,1);  % Cumulative acceleration

%% Fuzzy Velocity Controller 
e_vel_prev = zeros(6,1); 
max_de_error = 100;      
max_vel_error = 0.1;     
dt = 0.001;              

%% Joint Angle Plot Initialization - Real-time visualization
figure(5);
set(gcf, 'Color', 'w');
xlabel('\textit{iteration}','Interpreter','latex','FontSize',20);
ylabel('\textit{angle}/\textit{deg}','Interpreter','latex','FontSize',20);
hold on;

% Pre-create empty lines for real-time update (avoid re-drawing)
q1_line = plot(NaN, NaN, 'Color', colors(1,:), 'LineWidth', 2);
q2_line = plot(NaN, NaN, 'Color', colors(2,:), 'LineWidth', 2);
q3_line = plot(NaN, NaN, 'Color', colors(3,:), 'LineWidth', 2);
q4_line = plot(NaN, NaN, 'Color', colors(4,:), 'LineWidth', 2);
q5_line = plot(NaN, NaN, 'Color', colors(5,:), 'LineWidth', 2);
q6_line = plot(NaN, NaN, 'Color', colors(6,:), 'LineWidth', 2);

% Add legend for joint angle plot (LaTeX formatted)
qLegend = legend([q1_line, q2_line, q3_line, q4_line, q5_line, q6_line], ...
    {'$\textit{joint1\ angle}$', '$\textit{joint2\ angle}$', '$\textit{joint3\ angle}$', ...
     '$\textit{joint4\ angle}$', '$\textit{joint5\ angle}$', '$\textit{joint6\ angle}$'}, ...
    'Location', 'northeastoutside', ...
    'Interpreter', 'latex');
set(qLegend, 'FontSize', 12);

%% Trajectory Data Storage - Struct for detailed dynamics
real_traj = struct(...
    'time', [], ...
    'positions', [], ...
    'velocities', [], ...
    'accelerations', [], ...
    'jerks', []);
% Per-joint dynamics storage
joint_dynamics = struct();
for i = 1:6
    joint_dynamics(i).position = [];
    joint_dynamics(i).velocity = [];
    joint_dynamics(i).acceleration = [];
    joint_dynamics(i).jerk = [];
end
x_angle = []; % Independent time axis for angle plot
joint1_angle = []; joint2_angle = []; joint3_angle = [];
joint4_angle = []; joint5_angle = []; joint6_angle = [];

%% Main Control Loop - Visual servoing with MPC and Ruckig trajectory
prev_velocity = zeros(6,1); % Initial joint velocity (0 for all joints)
for k = 1:N-1
    % Set target point in task space (fixed position for servoing)
    X_s_t_set = [0.3, 0, height + 0.1]';
    % Define 4 feature points around target (small offset for IBVS)
    X_s_i_set = [X_s_t_set(1,1)-0.02  X_s_t_set(1,1)-0.02  X_s_t_set(1,1)+0.02   X_s_t_set(1,1)+0.02;
             X_s_t_set(2,1)-0.02  X_s_t_set(2,1)+0.02  X_s_t_set(2,1)-0.02   X_s_t_set(2,1)+0.02;
             X_s_t_set(3,1)-0.1   X_s_t_set(3,1)-0.1   X_s_t_set(3,1)-0.1    X_s_t_set(3,1)-0.1;];

    % Plot feature points in task space (red lines + stars)
    figure(1)
    line(X_s_i_set(1,[1 2]),X_s_i_set(2,[1 2]),X_s_i_set(3,[1 2]),'color','r');
    line(X_s_i_set(1,[2 4]),X_s_i_set(2,[2 4]),X_s_i_set(3,[2 4]),'color','r');
    line(X_s_i_set(1,[4 3]),X_s_i_set(2,[4 3]),X_s_i_set(3,[4 3]),'color','r');
    line(X_s_i_set(1,[3 1]),X_s_i_set(2,[3 1]),X_s_i_set(3,[3 1]),'color','r');
    plot3(X_s_i_set(1,1),X_s_i_set(2,1),X_s_i_set(3,1),'r*')
    plot3(X_s_i_set(1,2),X_s_i_set(2,2),X_s_i_set(3,2),'r*')
    plot3(X_s_i_set(1,3),X_s_i_set(2,3),X_s_i_set(3,3),'r*')
    plot3(X_s_i_set(1,4),X_s_i_set(2,4),X_s_i_set(3,4),'r*')
    plot3(X_s_t_set(1), X_s_t_set(2), X_s_t_set(3)-0.1, 'r+', 'LineWidth', 1);
    hold on ;
    
    % Extract current end-effector position and camera pose
    X_s_e_current = T_s_e_current([1 2 3],4);
    camera_current = eMc*T_s_e_current;
    T_camera_current = camera_current([1 2 3],4);
    x = [x,k]; % Update iteration counter
    
    % Initialize visual feature at first iteration
    if k==1
        T_camera2base = eMc*T_s_e_current; % Camera to base transformation
        T_c_s = inv(T_camera2base);        % Inverse for 3D->2D projection
        current_pixel = Camera_3to2(X_s_i_set,T_c_s); % Project 3D points to pixel
        s_history(:,k) = Calculate_s(current_pixel,K); % Store initial feature vector
    end
    
    % Get current visual feature and previous control input
    s = s_history(:,k);
    if k>1
        tau_now = tau_history(:,k-1); % Use previous control input for MPC initial guess
    else
        tau_now = 0*ones(6,1); % Zero initial control input
    end
    
    %% IBVS Jacobian Calculation - Image-Based Visual Servoing core
    Ls = Calculate_Ls(s,z_star); 
    Ls_inverse = pinv(Ls);       
    Z = z_star ;                 
    Ls0 = Calculate_Ls(s,Z);     
    B = Ts*Ls0;                  
    A = eye(size(s,1));          
    v_max = 15; w_max = 15;      
    
    % Generate QP cost function for MPC
    [H,f] = generate_qp_cost(s,s_target,A,B,Q,R,Np);
    % Initialize feature error at first iteration
    if k==1
        s_next = Calculate_s(current_pixel,K);
        e_current = s_next-s_target;
    end
    
    %% MPC Controller - QP-based MPC for visual servoing
    V_camera = -Lambda*Ls_inverse*e_current;
    %V_camera = mpc_controller_qp(s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,image_width,image_height,tau_now,A,B,v_max,w_max);
    %V_camera = mpc_controller(s,s_target,z_star,Np,Q,R_weight,W_weight,Ts,K,F,q_current,image_width,image_height,tau_now,fig_handle,vis_predict,A,B);
    %V_camera = mpc_controller(s,s_target,camera_current(3,4),Np,Q,R_weigth,Ts,K,F,image_width,image_heigt,tau_now,fig_handle,vis_predict);
    tau_history(:,k) = V_camera; 
    
    %% Velocity Mapping - Camera velocity -> End-effector velocity
    R_eMc = eMc(1:3,1:3); 
    p_eMc = eMc(1:3,4);   
    V_end_effector = cross(p_eMc,R_eMc*V_camera(4:6))+R_eMc*V_camera(1:3);
    % Convert camera angular velocity to end-effector angular velocity
    omega_end_effector = R_eMc*V_camera(4:6);
    V_b_e = [ V_end_effector;omega_end_effector]; % End-effector twist (6x1)
    
    %% Inverse Kinematics - End-effector velocity -> Joint velocity
    joints_velocity = jr603_ivkine(V_b_e, theta_current, joint_velocity_max);
    disp('joints_velocity');
    disp(joints_velocity);

    %% Ruckig Trajectory Update - Jerk-limited velocity tracking
    [~,current_vellocity,current_acc,~] = ruckig_mex('get_state');
    next_velocity = joints_velocity;
    % Calculate velocity error (current vs target)
    e_vel = abs(next_velocity-current_vellocity);
    e_sum = e_sum+e_vel; % Accumulate velocity error
    disp('e_sum');
    disp(e_sum);
    disp('e_vel');
    disp(e_vel);
    % Normalize velocity error (0-1 range for fuzzy controller)
    e_vel_normalized = min(e_vel /max_vel_error, 1);

    % Calculate error derivative (rate of change)
    de_vel = (e_vel-e_vel_prev)/dt;
    de_vel_normalized = max(-1,min(1,de_vel/max_de_error)); % Clamp to [-1,1]
    disp('e_vel');
    disp(e_vel);
    disp('de_vel');
    disp(de_vel); 
    disp('e_vel_normalized');
    disp(e_vel_normalized);
    disp('de_vel_normalized');
    disp(de_vel_normalized);
    e_vel_prev = e_vel; % Update previous error

    %% Fuzzy Velocity Controller V3 - Adjust acc/jerk limits dynamically
    for joint_idx = 1:6
    [rucking_max_acc(joint_idx), rucking_max_jerk(joint_idx)] = ...
        fuzzy_velocity_controller_V3(e_vel_normalized(joint_idx),de_vel_normalized(joint_idx),a_rated);
    end

    % Update Ruckig target velocity/acceleration constraints
    disp('rucking_max_acc:')
    disp(rucking_max_acc);
    disp('rucking_max_jerk:')
    disp(rucking_max_jerk);
    ruckig_mex('update_targets',joints_velocity,zeros(1,6),rucking_max_acc,rucking_max_jerk);
    [~,current_vellocity,test_current_acc,test_current_jerk] = ruckig_mex('get_state');
    disp('test_current_acc:')
    disp(test_current_acc);
    disp('test_current_jerk:')
    disp(test_current_jerk);

    % Accumulate acceleration/jerk for performance analysis
    acc_sum = acc_sum+abs(test_current_acc);
    jerk_sum = jerk_sum+abs(test_current_jerk);
    disp('acc_sum:')
    disp(acc_sum);
    disp('jerk_sum:')
    disp(jerk_sum);

    % Check Ruckig trajectory completion status
    disp('target_joints_velocity:');
    disp(joints_velocity)
    is_finished = ruckig_mex('is_finished');
    disp('is_finished:');
    disp(is_finished);
    has_new_target = ruckig_mex('has_new_target');
    disp('has_new_target:');
    disp(has_new_target);
    
    %% Ruckig Step Execution - Generate smooth joint trajectory
    for j = 1:rucking_steps
        has_new_target = false;
        step_time = step_time+1;
        j = j+1;
        disp(j);
        disp('target_joints_velocity:');
        disp(joints_velocity)
        ruckig_mex('step');
        % Get current joint state (pos/vel/acc/jerk)
        [pos,vel,acc,jerk] = ruckig_mex('get_state');
        disp('current_pos:');
        disp(pos);
        disp('current_vel:');
        disp(vel);
        disp('current_acc:');
        disp(acc);
        disp('current_jerk');
        disp(jerk);

        % Update joint angles and plot robot pose
        theta_next = pos;
        disp('current_theta:')
        disp(theta_next)
        figure(1)
        JR603.plot(theta_next, 'workspace', [-1 1 -1 1 -1 1], 'delay', 0, 'view', [135 35], 'noarrow', 'nowrist');
        drawnow limitrate % Refresh plot (limit rate for performance)
        disp("Update figure window")
        
        % Record trajectory data (time/pos/vel/acc/jerk)
        current_time = 0.001*step_time;
        real_traj.time = [real_traj.time;current_time];
        real_traj.positions = [real_traj.positions;pos];
        real_traj.velocities = [real_traj.velocities;vel];
        real_traj.accelerations = [real_traj.accelerations;acc];
        real_traj.jerks = [real_traj.jerks;jerk];
        % Store per-joint dynamics
        for joint_idx = 1:6
            joint_dynamics(joint_idx).position = [joint_dynamics(joint_idx).position; pos(joint_idx)];
            joint_dynamics(joint_idx).velocity = [joint_dynamics(joint_idx).velocity; vel(joint_idx)];
            joint_dynamics(joint_idx).acceleration = [joint_dynamics(joint_idx).acceleration; acc(joint_idx)];
            joint_dynamics(joint_idx).jerk = [joint_dynamics(joint_idx).jerk; jerk(joint_idx)];
        end
        
        % Update forward kinematics with new joint angles
        T_s_e_current = jr603_fkine(theta_next);
        X_s_e_current = T_s_e_current([1 2 3], 4);
        camera_current = eMc * T_s_e_current;
        T_camera_current = camera_current([1 2 3], 4);

        % Record joint angles (convert rad to deg for visualization)
        joint1_angle = [joint1_angle, theta_current(1)*180/pi];
        joint2_angle = [joint2_angle, theta_current(2)*180/pi];
        joint3_angle = [joint3_angle, theta_current(3)*180/pi];
        joint4_angle = [joint4_angle, theta_current(4)*180/pi];
        joint5_angle = [joint5_angle, theta_current(5)*180/pi];
        joint6_angle = [joint6_angle, theta_current(6)*180/pi];

        % Extend iteration axis for angle plot
        x_angle = [x_angle, k];
    
        % Check if Ruckig trajectory is finished
        is_finished = ruckig_mex('is_finished');
        disp('is_finished:')
        disp(is_finished)        
        if is_finished
            disp('j:')
            disp(j);
            j = 0;
            disp("Velocity command completed")
        end

        disp("Execute step():j=")
        disp(j);
    end
    
    %% Update Joint Angles - Set current pose to Ruckig output
    [pos_,vel_,acc_,jerk_] = ruckig_mex('get_state');
    theta_current = pos_;

    % Re-calculate end-effector and camera pose
    T_s_e_current = jr603_fkine(theta_current);
    camera_current = eMc*T_s_e_current;
    T_camera_current = camera_current([1 2 3],4);

    % Camera to base transformation and 3D->2D projection
    T_camera2base = eMc*T_s_e_current;
    camera_position = T_camera2base(1:3,4); % Camera position (base frame)
    T_c_s = inv(T_camera2base);  % Inverse transform for projection
    current_pixel = Camera_3to2(X_s_i_set,T_c_s); % Project 3D points to pixel
    % Calculate center of current feature points
    current_center_x = mean(current_pixel(1,:));
    current_center_y = mean(current_pixel(2,:));
    if k==1
        initial_center_x = current_center_x;
        initial_center_y = current_center_y;
    end
    
    %% Camera Image Plot Update - Current feature points
    figure(2)
    plot(current_pixel(1,1),current_pixel(2,1),'r.')
    plot(current_pixel(1,2),current_pixel(2,2),'r.')
    plot(current_pixel(1,3),current_pixel(2,3),'r.')
    plot(current_pixel(1,4),current_pixel(2,4),'r.')
    plot(current_center_x,current_center_y,'b.') % Current center (blue dot)
    % Plot line between desired and current center (green)
    line([target_center_x,current_center_x],[target_center_y,current_center_y],'color','g');
    % Draw feature point contour at first iteration
    if k == 1
        line(current_pixel(1,[1 2]),current_pixel(2,[1 2]),'color','r');
        line(current_pixel(1,[2 4]),current_pixel(2,[2 4]),'color','r');
        line(current_pixel(1,[4 3]),current_pixel(2,[4 3]),'color','r');
        line(current_pixel(1,[3 1]),current_pixel(2,[3 1]),'color','r');
    end

    %% Feature Error Calculation - Update visual servoing error
    s_next = Calculate_s(current_pixel,K); % New feature vector
    s_history(:,k+1) = s_next;    % Store feature history
    e_current = s_next - s_target; % Feature error (tracking error)
    disp('e_current:');
    disp(e_current);
    e_current_norm=norm(e_current,2); % L2 norm of error
    disp('e_current_norm:');
    disp(e_current_norm);
    
    %% Termination Check - Stop if error is below threshold
    if e_current_norm < error_max
        % Plot detailed trajectory dynamics (pos/vel/acc/jerk per joint)
        traj_fig = figure('Color', 'w');
        set(traj_fig, 'Position', [100, 100, 1200, 800]);

        for joint_idx = 1:6
            % Position plot
            subplot(4,6, joint_idx)
            plot(real_traj.time, joint_dynamics(joint_idx).position, 'Color',colors(1,:), 'LineWidth', 1.5)
            title(['\textit{Joint }', '$', num2str(joint_idx), '$', '\textit{ Pos(rad)}'], ...
            'Interpreter','latex','FontSize',10);
            xlabel('\textit{t(s)}', 'Interpreter','latex','FontSize',10);
            grid on
    
            % Velocity plot
            subplot(4,6, joint_idx+6)
            plot(real_traj.time, joint_dynamics(joint_idx).velocity,'Color', colors(2,:), 'LineWidth', 1.5)
            title(['\textit{Joint }', '$', num2str(joint_idx), '$', '\textit{ Vel(rad/s)}'], ...
            'Interpreter','latex','FontSize',10);
            xlabel('\textit{t(s)}', 'Interpreter','latex','FontSize',10);
            grid on
   
            % Acceleration plot
            subplot(4,6, joint_idx+12)
            plot(real_traj.time, joint_dynamics(joint_idx).acceleration,'Color',colors(3,:), 'LineWidth', 1.5)
            title(['\textit{Joint ', num2str(joint_idx), ' Acc(rad/s$^2$)}'], ...
            'Interpreter','latex','FontSize',10);
            xlabel('\textit{t(s)}', 'Interpreter','latex','FontSize',10);
            grid on
    
            % Jerk plot
            subplot(4,6, joint_idx+18)
            plot(real_traj.time, joint_dynamics(joint_idx).jerk,'Color',colors(4,:), 'LineWidth', 1.5)
            title(['\textit{Joint ', num2str(joint_idx), ' Jerk(rad/s$^3$)}'], ...
            'Interpreter','latex','FontSize',10);
            xlabel('\textit{t(s)}', 'Interpreter','latex','FontSize',10);
            grid on
        end
        break % Exit main loop if error is within threshold
    end

    %% Normalized Image Plane Plot - For feature error visualization
    s_target_2D = [s_target(1:2:end)'; s_target(2:2:end)']; % Reshape to 2x4
    s_current_2D = [s_next(1:2:end)'; s_next(2:2:end)'];
    s_first_2D = [s(1:2:end)'; s(2:2:end)'];
    s_target_center = mean(s_target_2D, 2); % Desired center (normalized)
    s_current_center = mean(s_current_2D, 2); % Current center (normalized)
    
    figure(8)
    set(gcf, 'Color', 'w');
    xlabel('$\textit{u}$', 'FontSize', 20, 'Interpreter', 'latex');
    ylabel('$\textit{v}$', 'FontSize', 20, 'Interpreter', 'latex');
    axis equal
    xlim([-1,1]) % Normalized pixel range
    ylim([-1,1])
    hold on
    % Plot field of view boundary (dashed black line)
    u_view = [-0.8, 0.8, 0.8, -0.8, -0.8];
    v_view = [-0.8, -0.8, 0.8, 0.8, -0.8];
    plot(u_view, v_view, 'k--', 'LineWidth', 1.5)
    % Plot desired feature points and contour
    plot(s_target_2D(1,:),s_target_2D(2,:),'b*')
    line(s_target_2D(1,[1 2]), s_target_2D(2,[1 2]), 'Color', 'b')
    line(s_target_2D(1,[2 4]), s_target_2D(2,[2 4]), 'Color', 'b')
    line(s_target_2D(1,[4 3]), s_target_2D(2,[4 3]), 'Color', 'b')
    line(s_target_2D(1,[3 1]), s_target_2D(2,[3 1]), 'Color', 'b');
    % Plot current feature points and contour (first iteration only)
    if k == 1
        line(s_current_2D(1,[1 2]),s_current_2D(2,[1 2]),'color','r');
        line(s_current_2D(1,[2 4]),s_current_2D(2,[2 4]),'color','r');
        line(s_current_2D(1,[4 3]),s_current_2D(2,[4 3]),'color','r');
        line(s_current_2D(1,[3 1]),s_current_2D(2,[3 1]),'color','r');
    end
    % Plot current feature points (red dots)
    plot(s_current_2D(1,[1 2]),s_current_2D(2,[1 2]),'r.')
    plot(s_current_2D(1,[2 4]),s_current_2D(2,[2 4]),'r.')
    plot(s_current_2D(1,[4 3]),s_current_2D(2,[4 3]),'r.')
    plot(s_current_2D(1,[3 1]),s_current_2D(2,[3 1]),'r.')
    % Plot center points and error line
    plot(s_current_center(1),s_current_center(2),'b.')
    plot(s_target_center(1),s_target_center(2),'b.')
    line([s_target_center(1), s_current_center(1)], [s_target_center(2), s_current_center(2)], 'Color', 'g')
   
    %% Feature Error Plot - Per-point error history
    x1_e = [x1_e,e_current(1)]; y1_e = [y1_e,e_current(2)];
    x2_e = [x2_e,e_current(3)]; y2_e = [y2_e,e_current(4)];
    x3_e = [x3_e,e_current(5)]; y3_e = [y3_e,e_current(6)];
    x4_e = [x4_e,e_current(7)]; y4_e = [y4_e,e_current(8)];
    
    figure(3)
    set(gcf, 'Color', 'w');
    hold on
    % Plot error for each feature point (u/v direction)
    h1=plot(x, x1_e, 'Color', colors(1,:), 'LineStyle', '-','LineWidth', 2);
    h2=plot(x , y1_e, 'Color', colors(1,:), 'LineStyle', '--','LineWidth', 2);
    h3=plot(x, x2_e, 'Color', colors(2,:), 'LineStyle', '-','LineWidth', 2);
    h4=plot(x, y2_e, 'Color', colors(2,:), 'LineStyle', '-- ','LineWidth', 2);
    h5=plot(x, x3_e, 'Color', colors(3,:), 'LineStyle', '-','LineWidth', 2);
    h6=plot(x, y3_e, 'Color', colors(3,:), 'LineStyle', '--','LineWidth', 2);
    h7=plot(x, x4_e, 'Color', colors(4,:), 'LineStyle', '-','LineWidth', 2);
    h8=plot(x, y4_e, 'Color', colors(4,:), 'LineStyle', '--','LineWidth', 2);

    % Label and legend for error plot
    xlabel('\textit{Number of iterations}', 'Interpreter', 'latex', 'FontSize', 20);
    ylabel('\textit{Feature error}', 'Interpreter', 'latex', 'FontSize', 20);
    hLegend = legend([h1, h2, h3, h4, h5, h6, h7, h8], ...
    {'$eu1$', '$ev1$', '$eu2$', '$ev2$', ...
     '$eu3$', '$ev3$', '$eu4$', '$ev4$'}, ...
    'Location', 'northeastoutside', ...
    'Interpreter', 'latex', 'FontSize', 12);

   %% Camera Velocity Plot - End-effector velocity history
   disp('v_camera:');
   disp(V_b_e);
   x_v =  [x_v ,V_b_e(1)]; y_v =  [y_v ,V_b_e(2)]; z_v =  [z_v ,V_b_e(3)];
   rx_v = [rx_v,V_b_e(4)]; ry_v = [ry_v,V_b_e(5)]; rz_v = [rz_v,V_b_e(6)];
   
   figure(4)
   set(gcf, 'Color', 'w');
   xlabel('\textit{Number of iterations}', 'Interpreter', 'latex', 'FontSize', 20);
   ylabel('\textit{Camera~velocity}', 'Interpreter', 'latex', 'FontSize', 20);
   hold on
   
   % Plot linear/angular velocity components
   h_xv=plot(x, x_v, 'Color',   colors(1,:), 'LineStyle', '-','LineWidth', 2);
   h_yv=plot(x, y_v, 'Color',   colors(2,:), 'LineStyle', '-','LineWidth', 2);
   h_zv=plot(x, z_v, 'Color',   colors(3,:), 'LineStyle', '-','LineWidth', 2);
   h_rxv=plot(x, rx_v, 'Color', colors(4,:), 'LineStyle', '-','LineWidth', 2);
   h_ryv=plot(x, ry_v, 'Color', colors(5,:), 'LineStyle', '-','LineWidth', 2);
   h_rzv=plot(x, rz_v, 'Color', colors(6,:), 'LineStyle', '-','LineWidth', 2);
   
   % Legend for velocity plot
   hLegend = legend([h_xv, h_yv, h_zv, h_rxv, h_ryv, h_rzv], ...
    {'$v_x$', '$v_y$', '$v_z$', ...
     '$\omega_x$', '$\omega_y$', '$\omega_z$'}, ...
    'Location', 'northeastoutside', 'Interpreter', 'latex');
   set(hLegend, 'FontSize', 12);

   %% Update Joint Angle Plot - Real-time refresh
   set(q1_line, 'XData', x_angle, 'YData', joint1_angle);
   set(q2_line, 'XData', x_angle, 'YData', joint2_angle);
   set(q3_line, 'XData', x_angle, 'YData', joint3_angle);
   set(q4_line, 'XData', x_angle, 'YData', joint4_angle);
   set(q5_line, 'XData', x_angle, 'YData', joint5_angle);
   set(q6_line, 'XData', x_angle, 'YData', joint6_angle);

   %% Joint Velocity Plot - History of joint velocity
   joint1_velocity = [joint1_velocity,joints_velocity(1)];
   joint2_velocity = [joint2_velocity,joints_velocity(2)];
   joint3_velocity = [joint3_velocity,joints_velocity(3)];
   joint4_velocity = [joint4_velocity,joints_velocity(4)];
   joint5_velocity = [joint5_velocity,joints_velocity(5)];
   joint6_velocity = [joint6_velocity,joints_velocity(6)];
   
   figure(6)
   set(gcf, 'Color', 'w');
   xlabel('$\textit{iteration}$', 'FontSize', 20, 'Interpreter', 'latex');
   ylabel('$\textit{joints\ velocity\ /\ (rad/s)}$', 'FontSize', 20, 'Interpreter', 'latex');
   hold on
   % Plot joint velocity for each axis
   v_joint1=plot(x, joint1_velocity, 'Color', colors(1,:), 'LineStyle', '-','LineWidth', 2);
   v_joint2=plot(x, joint2_velocity, 'Color', colors(2,:), 'LineStyle', '-','LineWidth', 2);
   v_joint3=plot(x, joint3_velocity, 'Color', colors(3,:), 'LineStyle', '-','LineWidth', 2);
   v_joint4=plot(x, joint4_velocity, 'Color', colors(4,:), 'LineStyle', '-','LineWidth', 2);
   v_joint5=plot(x, joint5_velocity, 'Color', colors(5,:), 'LineStyle', '-','LineWidth', 2);
   v_joint6=plot(x, joint6_velocity, 'Color', colors(6,:), 'LineStyle', '-','LineWidth', 2);
   
   % Legend for joint velocity plot
   hLegend = legend([v_joint1, v_joint2, v_joint3, v_joint4, v_joint5, v_joint6], ...
    {'$\textit{joint1\ velocity}$', '$\textit{joint2\ velocity}$', '$\textit{joint3\ velocity}$', ...
     '$\textit{joint4\ velocity}$', '$\textit{joint5\ velocity}$', '$\textit{joint6\ velocity}$'}, ...
    'Location', 'northeastoutside', 'Interpreter', 'latex');
   set(hLegend, 'FontSize', 12);

   %% End-effector Trajectory Plot - 3D path visualization
   T_s_e_current = jr603_fkine(theta_current);
   X_s_e_current = T_s_e_current([1 2 3],4);
   figure(1)
   set(gcf, 'Color', 'w');
   % Update robot pose in task space
   JR603.plot(theta_current, 'workspace', [-1 1 -1 1 -1 1], 'delay', 0, 'view', [135 35], 'noarrow', 'nowrist');
   Trajectory(:,k) = X_s_e_current; % Store end-effector position
   % Plot trajectory line (green) if more than one point
   if k> 1
       plot3(Trajectory(1,[k-1 k]),Trajectory(2,[k-1 k]),Trajectory(3,[k-1 k]),'color','g','LineWidth',1)
   end
end