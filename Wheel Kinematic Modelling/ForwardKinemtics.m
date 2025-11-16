%% Forward Kinematics + Visualization for Differential-Drive Robot
% Name: Tekumudi Vivek Sai Surya Chaitanya
% Matrikelnummer: 3759549

clear; clc; close all;

%% Robot parameters
r = 0.05;      % wheel radius [m]
d = 0.3;       % distance between wheels [m] (track width)
L = 0.4;       % robot body length [m]

%% Wheel velocities
vL = 0.1;      % left wheel linear velocity [m/s]
vR = 0.15;     % right wheel linear velocity [m/s]

%% Initial robot pose
xR = 0;        % x position [m]
yR = 0;        % y position [m]
theta = pi/4;  % heading angle [rad]

%% Simulation parameters
dt = 0.1;      % time step [s]
T = 5;         % total simulation time [s]
steps = T/dt;

%% Prepare figure for animation
figure('Color','w'); axis equal; grid on; hold on;
xlim([-1 2]); ylim([-1 2]);
xlabel('X [m]'); ylabel('Y [m]');
title('Differential-Drive Robot Simulation');

% Robot body rectangle definition (centered at robot reference)
body = [-L/2, -d/2;
         L/2, -d/2;
         L/2,  d/2;
        -L/2,  d/2]';

% Wheel rectangles (for visualization)
wheel_w = 0.02; % wheel width
wheel_l = 0.08; % wheel length
wheelL = [-wheel_l/2, -wheel_l/2, wheel_l/2, wheel_l/2;
           -wheel_w/2, wheel_w/2, wheel_w/2, -wheel_w/2];
wheelR = wheelL; % same size for right wheel

robot_patch = patch('XData',[], 'YData',[], 'FaceColor','b');
wheelL_patch = patch('XData',[], 'YData',[], 'FaceColor','k');
wheelR_patch = patch('XData',[], 'YData',[], 'FaceColor','k');

%% Simulation loop
for k = 1:steps
    % Forward Kinematics
    v = (vR + vL)/2;
    omega = (vR - vL)/d;

    xdot = v * cos(theta);
    ydot = v * sin(theta);
    thetadot = omega;

    % Update robot pose
    xR = xR + xdot*dt;
    yR = yR + ydot*dt;
    theta = theta + thetadot*dt;

    % Rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

    % Transform body and wheels to world frame
    body_world = R*body + [xR; yR];
    
    % Left wheel position relative to robot center
    wheelL_pos = R*(wheelL + [0; -d/2]) + [xR; yR];
    wheelR_pos = R*(wheelR + [0; d/2]) + [xR; yR];

    % Update patches
    set(robot_patch, 'XData', body_world(1,:), 'YData', body_world(2,:));
    set(wheelL_patch, 'XData', wheelL_pos(1,:), 'YData', wheelL_pos(2,:));
    set(wheelR_patch, 'XData', wheelR_pos(1,:), 'YData', wheelR_pos(2,:));

    drawnow;
    pause(dt*0.5); % slow down for visualization
end
