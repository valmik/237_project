function [q_opt, u_opt] = unicycle_MPC(N, q0, q_expected, u_expected, ts)
%UNICYCLE_MPC Summary of this function goes here
%   Detailed explanation goes here


q = sdpvar(3, N+1);
u = sdpvar(2, N);

%% Initial conditions
assign(q, q_expected);
assign(u, u_expected);

%% MPC Constraints

constraints = [];

% Dynamics
for i = 1:N
    constraints = [constraints, ...
        [q(:, i+1) == unicycle_dynamics(q(:,i), u(:,i), ts)]:'dynamics'];
end

% Start state
constraints = [constraints, ...
    [q(:,1) == q0]:'initial state'];


%% MPC Cost
R = 0.5; % Arbitrary

cost = sum(sum((q - q_expected).^2)); % + R*sum(sum((u - u_expected).^2));

%% Solve

options = sdpsettings('verbose', true, 'solver', 'IPOPT');
feas = optimize(constraints, cost, options)

q_opt = double(q);
u_opt = double(u);

end

