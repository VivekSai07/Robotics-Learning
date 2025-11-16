function visualize_arc(x, b, N)
% Visualize ddmodel over N small steps

figure; hold on; axis equal; grid on;
title("ddmodel visualization (arc integration)");
sL = x(4);
sR = x(5);
state = x;
path = zeros(N+1, 2);
path(1,:) = state(1:2)';

for i = 1:N
    state = [state(1:3); sL; sR];  % ensure 5 elements
    state = ddmodel(state, b);
    path(i+1,:) = state(1:2)';
end

plot(path(:,1), path(:,2), 'LineWidth', 2);
plot(path(1,1), path(1,2), 'go', 'MarkerSize', 10);
plot(path(end,1), path(end,2), 'rx', 'MarkerSize', 12);

legend("Trajectory","Start","End");
xlabel("x"); ylabel("y");
end
