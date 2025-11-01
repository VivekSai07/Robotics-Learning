%% Differential Drive Robot Visualization
clear; clc; close all;

% Parameters
r = 0.05;        % wheel radius [m]
b = 0.15;        % half of wheelbase [m]
dt = 0.05;       % time step [s]
T  = 20;         % total simulation time [s]
N  = T/dt;

% Initial state: [x; y; theta; phi_L; phi_R]
q = [0; 0; 0; 0; 0];

% Controls: [u_L; u_R] (rad/s)
uL = 10;   % left wheel angular velocity
uR = 12;   % right wheel angular velocity

% | Variable     | Meaning                | Effect          |
% | :----------- | :--------------------- | :-------------- |
% | (u_L = u_R)  | Both wheels same speed | Straight motion |
% | (u_L > u_R)  | Left wheel faster      | Turn right      |
% | (u_L < u_R)  | Right wheel faster     | Turn left       |
% | (u_L = -u_R) | Opposite speeds        | Spin in place   |

% Storage for trajectory
traj = zeros(3,N);

figure;
for k = 1:N
    % Compute derivatives from kinematics
    qdot = [ (r/2)*cos(q(3))   (r/2)*cos(q(3));
             (r/2)*sin(q(3))   (r/2)*sin(q(3));
             (-r/(b))          (r/(b));
             1                 0;
             0                 1 ] * [uL; uR];
         
    % Integrate
    q = q + qdot * dt;
    traj(:,k) = q(1:3);
    
    % Visualization
    clf;
    plot(traj(1,1:k), traj(2,1:k), 'b-', 'LineWidth', 1.5); hold on;
    draw_diffdrive(q(1), q(2), q(3), q(4), q(5), r, b);
    axis equal; grid on;
    xlim([-1.5 1.5]); ylim([-1.5 1.5]);
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Differential Drive (t=%.2fs)', k*dt));
    drawnow;
end

%% --- Helper function to draw differential drive ---
function draw_diffdrive(x, y, theta, phiL, phiR, r, b)
    % Robot chassis rectangle
    L = 2*b; W = 0.1;  % body dimensions
    Rb = [ L/2 -L/2 -L/2  L/2 L/2;
           W/2  W/2 -W/2 -W/2 W/2 ];
    Rmat = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    Rw = Rmat*Rb + [x; y];
    fill(Rw(1,:), Rw(2,:), [0.8 0.8 0.8], 'EdgeColor', 'k');

    % Wheel positions
    left_center  = [x; y] + Rmat*[0;  b];
    right_center = [x; y] + Rmat*[0; -b];
    ang = linspace(0, 2*pi, 30);
    wx = r*cos(ang); wy = r*sin(ang);

    % Left wheel
    wL = Rmat*[wx; wy];
    fill(wL(1,:)+left_center(1), wL(2,:)+left_center(2), 'k');
    % Wheel rotation indicator
    line(left_center(1)+[0, r*cos(phiL+pi/2+theta)], ...
         left_center(2)+[0, r*sin(phiL+pi/2+theta)], 'Color','r','LineWidth',1.5);

    % Right wheel
    wR = Rmat*[wx; wy];
    fill(wR(1,:)+right_center(1), wR(2,:)+right_center(2), 'k');
    line(right_center(1)+[0, r*cos(phiR+pi/2+theta)], ...
         right_center(2)+[0, r*sin(phiR+pi/2+theta)], 'Color','r','LineWidth',1.5);

    % Heading arrow
    quiver(x, y, 0.2*cos(theta), 0.2*sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
end
