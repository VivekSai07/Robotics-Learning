clear; clc; close all;

% --- Link lengths ---
L1 = 1.0;
L2 = 0.8;

% --- Motion setup ---
theta1_vals = linspace(0, pi/4, 100);
theta2_vals = linspace(0, pi/3, 100);

% --- Figure setup ---
figure('Color','w');
hold on; grid on; axis equal;
xlim([-2 2]); ylim([-2 2.5]);
xlabel('X'); ylabel('Y');
title('2-Link Planar Manipulator - Forward Kinematics Visualization');

% --- Initialize graphics objects ---
armLine = plot([0 0 0], [0 0 0], '-o', 'LineWidth', 2, 'MarkerSize', 8);
textInfo = text(-1.8, 1.6, '', 'FontSize', 12, 'FontWeight', 'bold', 'FontName','Consolas');

for k = 1:length(theta1_vals)
    % Current joint angles
    t1 = theta1_vals(k);
    t2 = theta2_vals(k);

    % --- Forward kinematics ---
    x1 = L1*cos(t1);
    y1 = L1*sin(t1);
    x2 = x1 + L2*cos(t1 + t2);
    y2 = y1 + L2*sin(t1 + t2);

    % --- Update arm position ---
    set(armLine, 'XData', [0 x1 x2], 'YData', [0 y1 y2]);

    % --- Update text info ---
    msg = sprintf(['θ₁ = %5.1f°\nθ₂ = %5.1f°\n\n' ...
                   'End-Effector:\nX = %6.3f\nY = %6.3f'], ...
                   rad2deg(t1), rad2deg(t2), x2, y2);
    set(textInfo, 'String', msg, 'Position', [x2+0.2, y2+0.2]);
    drawnow;
    pause(0.05);
end
