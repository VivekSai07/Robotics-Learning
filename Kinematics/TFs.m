%% Transformation Frames Visualization - 2-Link Planar Manipulator
clear; clc; close all;

% --- Link lengths ---
L1 = 1.0;
L2 = 0.8;

% --- Joint angles ---
theta1 = deg2rad(45);
theta2 = deg2rad(30);

% --- Transformation Matrices ---
T01 = [cos(theta1) -sin(theta1) L1*cos(theta1);
       sin(theta1)  cos(theta1) L1*sin(theta1);
       0            0           1];

T12 = [cos(theta2) -sin(theta2) L2*cos(theta2);
       sin(theta2)  cos(theta2) L2*sin(theta2);
       0            0           1];

T02 = T01 * T12;

% --- Extract positions ---
O0 = [0; 0];
O1 = T01(1:2,3);
O2 = T02(1:2,3);

% --- Plot setup ---
figure('Color','w'); hold on; grid on; axis equal;
xlim([-2 2]); ylim([-2 2]);
xlabel('X'); ylabel('Y');
title('Transformation Frames Visualization');

% --- Plot robot links ---
plot([O0(1) O1(1) O2(1)], [O0(2) O1(2) O2(2)], '-o', 'LineWidth', 2);

% --- Draw coordinate frames ---
drawFrame(eye(3), 0.2, 'Base {0}');
drawFrame(T01, 0.2, '{1}');
drawFrame(T02, 0.2, '{2}');

% --- Display transformation matrices ---
disp('T01 ='); disp(T01);
disp('T12 ='); disp(T12);
disp('T02 ='); disp(T02);

%% --- Function to draw coordinate frame ---
function drawFrame(T, scale, labelText)
    origin = T(1:2,3);
    xAxis = origin + scale * T(1:2,1);
    yAxis = origin + scale * T(1:2,2);
    
    % Draw axes
    plot([origin(1) xAxis(1)], [origin(2) xAxis(2)], 'r', 'LineWidth', 2); % X-axis
    plot([origin(1) yAxis(1)], [origin(2) yAxis(2)], 'b', 'LineWidth', 2); % Y-axis
    
    % Label
    text(origin(1)+0.05, origin(2)+0.05, labelText, 'FontSize', 10, 'FontWeight', 'bold');
end
