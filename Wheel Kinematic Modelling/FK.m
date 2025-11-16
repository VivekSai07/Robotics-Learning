%% Static Visualization of Differential-Drive Robot with Point P
% Name: Tekumudi Vivek Sai Surya Chaitanya
% Matrikelnummer: 3759549

clear; clc; close all;

%% Robot parameters
d = 0.3;      % distance between wheels [m]
L = 0.4;      % robot body length [m]

%% Robot pose in world frame
xR = 0.5;     
yR = 0.5;
theta = pi/4;

%% Robot body rectangle (centered at robot reference)
body = [-L/2, -L/2, L/2, L/2;
        -d/2, d/2, d/2, -d/2];

%% Rotation matrix
Rmat = [cos(theta) -sin(theta);
        sin(theta)  cos(theta)];

%% Transform body to world frame
body_world = Rmat * body + [xR; yR];

%% Point P in robot frame
xP_R = 0.3;   % x offset from robot center
yP_R = 0.1;   % y offset from robot center

% Transform P to world frame
P_world = [xR; yR] + Rmat * [xP_R; yP_R];

%% Plot
figure('Color','w'); hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Differential-Drive Robot with End-Effector P');

% Robot body
patch(body_world(1,:), body_world(2,:), 'b', 'FaceAlpha',0.5);
% Wheels as small rectangles
wheel_w = 0.02; wheel_l = 0.08;
wheelL = [-wheel_l/2, -wheel_l/2, wheel_l/2, wheel_l/2;
           -wheel_w/2, wheel_w/2, wheel_w/2, -wheel_w/2];
wheelR = wheelL;
wheelL_world = Rmat * (wheelL + [0; -d/2]) + [xR; yR];
wheelR_world = Rmat * (wheelR + [0; d/2]) + [xR; yR];
patch(wheelL_world(1,:), wheelL_world(2,:), 'k');
patch(wheelR_world(1,:), wheelR_world(2,:), 'k');

% Plot point P
plot(P_world(1), P_world(2), 'ro', 'MarkerSize',10, 'LineWidth',2);
text(P_world(1)+0.02, P_world(2)-0.02, 'P', 'FontSize',12, 'Color','r');

% Show robot reference point
plot(xR, yR, 'ks', 'MarkerSize',8, 'MarkerFaceColor','k');
text(xR+0.02, yR-0.02, 'R', 'FontSize',12);

%% Draw TF axes
axis_length = 0.15;

% Robot frame axes at R
quiver(xR, yR, axis_length*cos(theta), axis_length*sin(theta), 'r', 'LineWidth',2,'MaxHeadSize',0.5);
quiver(xR, yR, -axis_length*sin(theta), axis_length*cos(theta), 'g', 'LineWidth',2,'MaxHeadSize',0.5);
text(xR + 0.05, yR + 0.12, 'X_R', 'Color','r','FontSize',12);
text(xR - 0.12, yR + 0.05, 'Y_R', 'Color','g','FontSize',12);

% End-effector frame axes at P
thetaP = theta; % assuming P frame aligned with robot frame
quiver(P_world(1), P_world(2), axis_length*cos(thetaP), axis_length*sin(thetaP), 'r--', 'LineWidth',2,'MaxHeadSize',0.5);
quiver(P_world(1), P_world(2), -axis_length*sin(thetaP), axis_length*cos(thetaP), 'g--', 'LineWidth',2,'MaxHeadSize',0.5);
text(P_world(1)+0.05, P_world(2)+0.12, 'X_P', 'Color','r','FontSize',12);
text(P_world(1)-0.12, P_world(2)+0.05, 'Y_P', 'Color','g','FontSize',12);

xlim([0 1]); ylim([0 1]);