%% Setup Stuff

clear;
close all;
clc;

% Add IPOPT and other potential packages
setup_paths

%% Define an arbitrary path

ts = 0.01;

t = 0:ts:2;
u_expected = [sin(t*pi); cos(t*pi)];
q0_expected = [0;0;0];
q_expected = zeros(3, numel(t));
q_expected(:,1) = q0_expected;

for index = 1:(numel(t)-1)
    q_expected(:, index+1) = unicycle_dynamics(q_expected(:, index), u_expected(:, index), ts);
end

plot(q_expected(1,:), q_expected(2,:))


%% Do MPC

% Time horizon
N = 10;

% Pad expected path
q_expected = [q_expected, repmat(q_expected(:,end), [1, N+1])];
u_expected = [u_expected, zeros(2, N)];

q0 = [0; -0.05; 0];

q = zeros(3, numel(t));
q(:,1) = q0;

u = zeros(2, numel(t));

figure()
hold on;

for index = 1:(numel(t)-1)
    
    index
    
    [q_opt, u_opt] = unicycle_MPC(N, q(:,index), ...
        q_expected(:, index:(index + N)), ...
        u_expected(:, index:(index + N-1)), ts);
    
    plot(q_opt(1,:), q_opt(2,:), 'r');
    q(:,index+1) = unicycle_dynamics(q(:, index), u_opt(:, 1), ts);
end

plot(q(1,:), q(2,:), 'b')


















