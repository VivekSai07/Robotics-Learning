% Ex03_YourLastname_Matrikelnummer.m
% Mobile Robotics — Exercise 03 solutions (partial+code)
% Author: YourName
% Matrikelnummer: Matrikelnummer
%
% This script demonstrates:
%  - construction of rolling/no-slip constraints for a generic wheel set
%  - computation of mobility (delta_m, delta_s)
%  - computation of the Instantaneous Center of Rotation (ICR)
%
% Usage:
%  - edit the example 'wheels' struct below to match your robot
%  - run the script; results will be printed and plotted

clear; close all; clc;

fprintf('Ex03 script demo (replace header placeholders before submission)\n');

%% Example wheel set (you can edit these to match your robot)
% wheel struct fields:
%  .type  : 0 fixed wheel, 1 steerable wheel, 2/3 caster (passive), 4 swedish/omni
%  .pose  : [xw; yw; theta_w] where theta_w = atan2(yw,xw) + beta_w (sheet convention)
%  .params: [radius, width, offset, gamma] (only subset used here)

% Example: symmetric differential drive with two fixed wheels at y=+/- b/2
b = 0.5; % track width (m)
wheels = struct('type',{},'pose',{},'params',{});
% left wheel (fixed)
wheels(1).type = 0;
wheels(1).pose = [0; b/2; 0];   % x,y,theta_w (theta_w used as wheel pointing direction)
wheels(1).params = [0.05, 0, 0, 0]; % r, width, offset, gamma
% right wheel (fixed)
wheels(2).type = 0;
wheels(2).pose = [0; -b/2; 0];
wheels(2).params = [0.05, 0, 0, 0];

% Optional third fixed wheel example (uncomment to test overconstraint)
% wheels(3).type = 0;
% wheels(3).pose = [b/2; 0; pi/4];
% wheels(3).params = [0.05, 0, 0, 0];

%% Compute constraints and mobility
[J1, J2, C1, C2, delta_m, delta_s] = compute_wheel_constraints(wheels);

fprintf('Computed constraint matrices:\n');
fprintf('  J1 size: %d x 3\n', size(J1,1));
fprintf('  C1 size: %d x 3\n', size(C1,1));
fprintf('Mobility (delta_m) = %d, Steerability (delta_s) = %d\n', delta_m, delta_s);

%% Example ICR computation
q = [0; 0; 0];         % robot pose (x,y,theta)
qdot = [0.1; 0; 0.2];  % vx, vy, omega (body-frame velocities rotated by theta=0)
icr = compute_ICR(q, qdot);
if isfinite(icr(1))
    fprintf('ICR in world frame at: (%.3f, %.3f)\n', icr(1), icr(2));
else
    fprintf('ICR at infinity (pure translation)\n');
end

%% --- Fancy Robot Visualization -----------------------------------------

figure; hold on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Differential Drive Robot Visualization');
grid on;

% Compute robot body corners from wheel positions
wheel_coords = reshape([wheels.pose],3,[])';
x_coords = wheel_coords(:,1);
y_coords = wheel_coords(:,2);

margin = 0.1; % extra space around wheels
x_min = min(x_coords)-margin; x_max = max(x_coords)+margin;
y_min = min(y_coords)-margin; y_max = max(y_coords)+margin;

% Draw robot body rectangle
body_w = x_max - x_min;
body_h = y_max - y_min;
body_rect = rectangle('Position',[x_min, y_min, body_w, body_h],...
    'FaceColor',[0.9 0.9 0.9],'EdgeColor','k','LineWidth',2);

% Draw wheels as small rectangles
wheel_length = 0.08; % along pointing direction
wheel_width  = 0.04; % across pointing direction

for i=1:numel(wheels)
    w = wheels(i);
    xw = w.pose(1); yw = w.pose(2); theta = w.pose(3);
    
    % Wheel rectangle corners (centered at wheel)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    corners = 0.5*[-wheel_length -wheel_width;
                    wheel_length -wheel_width;
                    wheel_length  wheel_width;
                   -wheel_length  wheel_width]';
    corners = R * corners;
    corners(1,:) = corners(1,:) + xw;
    corners(2,:) = corners(2,:) + yw;
    fill(corners(1,:), corners(2,:), 'b');
    
    % Wheel heading arrow
    quiver(xw, yw, 0.05*cos(theta), 0.05*sin(theta), 'r', 'LineWidth',2, 'MaxHeadSize',1);
end

% Plot ICR if finite
if all(isfinite(icr))
    plot(icr(1), icr(2), 'go', 'MarkerSize',10,'MarkerFaceColor','g');
    text(icr(1)+0.02, icr(2)+0.02,'ICR','FontSize',12,'Color','g');
end

% Optional: plot robot velocity direction at center
q = [0; 0; 0];  % robot pose (from your example)
qdot = [0.1; 0; 0.2]; % body velocities
R_body = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))];
v_world = R_body * qdot(1:2);
quiver(q(1), q(2), v_world(1), v_world(2), 'm', 'LineWidth',2, 'MaxHeadSize',1);
text(q(1)+0.02, q(2)+0.02, 'v', 'Color','m','FontSize',12);

axis equal;

%% --- Local functions ----------------------------------------------------

function [J1, J2, C1, C2, delta_m, delta_s] = compute_wheel_constraints(wheels)
% compute_wheel_constraints  Build rolling/no-slip constraint matrices for a set of wheels.
% Inputs:
%   wheels - array of structs with fields:
%     .type, .pose = [x;y;theta_w], .params = [radius,width,offset,gamma]
% Outputs:
%   J1, J2 - (examples) pure rolling Jacobian blocks (rows per wheel)
%   C1, C2 - no-sliding constraint matrices (rows per wheel)
%   delta_m - degree of mobility (nullity of total C)
%   delta_s - degree of steerability (count of steerable wheels)

    n = numel(wheels);
    % We'll construct a constraint row for each wheel in the form:
    %   [a_i, b_i, c_i] * [xdot; ydot; thetadot] = 0
    % where (a_i,b_i) = direction components and c_i = moment term.
    rows = zeros(n,3);
    steerable_count = 0;
    for i=1:n
        w = wheels(i);
        xw = w.pose(1); yw = w.pose(2); thetaw = w.pose(3);
        % Interpret wheel pointing direction as the wheel rolling direction angle phi_i.
        phi = thetaw;  % user must provide pose.theta as appropriate per sheet convention
        switch w.type
            case 0 % fixed wheel (no steering)
                % The no-slip constraint forbids lateral velocity at contact:
                % row = [cos(phi+pi/2), sin(phi+pi/2), - (xw*sin(phi+pi/2) - yw*cos(phi+pi/2))]
                % equivalently set rolling axis direction
                a = cos(phi + pi/2);
                b = sin(phi + pi/2);
                c = - (xw*b - yw*a);
            case 1 % steerable wheel (we count steerable DOF)
                steerable_count = steerable_count + 1;
                a = cos(phi + pi/2);
                b = sin(phi + pi/2);
                c = - (xw*b - yw*a);
            case {2,3} % caster (passive) - approximate as no independent constraint (aligns)
                % For a perfect swiveling caster we do not add an independent non-holonomic constraint
                % (it passively aligns). We model by a row of zeros (no constraint).
                a = 0; b = 0; c = 0;
            case 4 % swedish/omni wheel (example)
                % For an omni wheel with rollers at gamma, the constraint is different.
                gamma = w.params(4);
                % simplified linearized row:
                a = cos(phi + gamma);
                b = sin(phi + gamma);
                c = - (xw*b - yw*a);
            otherwise
                a = 0; b = 0; c = 0;
        end
        rows(i,:) = [a, b, c];
    end

    % Remove zero rows (casters) for C1; for demonstration we treat J1==rows
    nz = any(abs(rows) > 1e-9,2);
    C1 = rows(nz,:);
    J1 = C1; % in many treatments J1==C1 for pure rolling; J2 reserved for other constraints
    % For demonstration we leave J2, C2 empty (could be used for lateral slip / extra)
    J2 = zeros(0,3);
    C2 = zeros(0,3);

    % degree of mobility = nullity of C_total (3 variables x,y,theta)
    C_total = C1; % if you have extra constraints stack here
    rankC = rank(C_total);
    delta_m = 3 - rankC;
    delta_s = steerable_count;
end

function icr = compute_ICR(q, qdot)
% compute_ICR  Compute ICR (x_icr,y_icr) in world frame from body velocities.
%  q = [x; y; theta]; qdot = [xdot; ydot; thetadot]
% If thetadot ~= 0, the ICR in body frame is at [-ydot/thetadot; xdot/thetadot] rotated to world.
    x = q(1); y = q(2); theta = q(3);
    vx = qdot(1); vy = qdot(2); omega = qdot(3);
    if abs(omega) < 1e-12
        icr = [Inf; Inf]; % ICR at infinity (pure translation)
        return;
    end
    % ICR in body frame:
    xb = -vy/omega;
    yb = vx/omega;
    % rotate to world:
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    world_icr = R * [xb; yb] + [x; y];
    icr = world_icr;
end
