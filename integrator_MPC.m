function [q_opt, u_opt] = integrator_MPC(N, q0, q_goal, ts)
%UNICYCLE_MPC Summary of this function goes here
%   Detailed explanation goes here


%% Dynamics

dynamics = @(q, u) q + [q(2); u]*ts;


%% Yalmip Setup

q = sdpvar(2, N+1);
u = sdpvar(1, N);

assign(u(1,1), 2);

%% Initial conditions
% assign(q, q_expected);
% assign(u, u_expected);

%% MPC Constraints

constraints = [];

% Dynamics
for i = 1:N
    constraints = [constraints, ...
        [q(:, i+1) == dynamics(q(:,i), u(:,i))]:'dynamics'];
end

% Start state
constraints = [constraints, ...
    [q(:,1) == q0]:'initial state'];


% CBF
for i = 1:N
    constraints = [constraints, ...
        [u(:,i) <= -(q(1,i) + q(2,i) - 1) - q(2,i)]:'CBF'];
end

% Input constraints
for i = 1:N
    constraints = [constraints, ...
        [abs(u(:,i)) <= 2]:'input constraints'];
end

%% MPC Cost
cost = 0.0*u*u' + sum((q(:, N+1) - q_goal)'*(q(:, N+1) - q_goal));

%% Solve

options = sdpsettings('verbose', true, 'solver', 'IPOPT');
feas = optimize(constraints, cost, options)

q_opt = double(q);
u_opt = double(u);

end

