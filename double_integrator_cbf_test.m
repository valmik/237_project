clear
close all
clc

%% Package paths

cur = pwd;
addpath( genpath( [cur, '/gen/' ] ));
rmpath( genpath( [cur, '/gen2/' ] ));


%% Define some dynamics and a basic controller

A = [zeros(2), eye(2); zeros(2), zeros(2)];
B = [0, 0; 0, 0; 1, 0; 0, 1];

K = place(A, B, [-3, -3, -5, -5]);

%% Problem Setup

x0 = [0;0;0;0];
x_goal = [1.5; 0.5; 0; 0];

ts = 0.1;
t = 0:ts:4;
x = zeros(4, numel(t));
x(:,1) = x0;

dynamics = @(x, u) x + (f_gen(x) + g_gen(x, u))*ts;

u = sdpvar(2,1);
uopt = zeros(2, numel(t));
kxopt = zeros(2, numel(t));

%% Dynamics and Optimization

for index = 1:numel(t)
    index
    
    kx = -K*(x(:,index) - x_goal);
    
    if norm(kx, 2) > 1
       kx = kx/norm(kx, 2); 
    end
    
    if x(1, index)==0 && x(2, index)==0
        assign(u, kx)
    else
        cost = 0.5*norm(u - kx, 2)^2;
        constraints = [];
        constraints = [cbf_constraint(x(:,index), u) >= 0];

        options = sdpsettings('verbose', true, 'solver', 'IPOPT');
        feas = optimize(constraints, cost, options);
    end

%     assign(u, kx)
    
    uopt(:,index) = double(u);
    kxopt(:,index) = kx;

    x(:, index+1) = dynamics(x(:,index), uopt(:,index));
end

%%

figure(1)
plot(x(1,:), x(2,:), cos(0:0.01:2*pi), sin(0:0.01:2*pi))
title('X position vs Y position')
xlabel('X, m')
ylabel('Y, m')
legend('Modified Trajectory', 'CBF constraint')

figure(2)
title('System state')
subplot(4,1,1)
plot(t, x(1,1:end-1))
ylabel('X, m')
subplot(4,1,2)
plot(t, x(2,1:end-1))
ylabel('Y, m')
subplot(4,1,3)
plot(t, x(3,1:end-1))
ylabel('dX, m/s')
subplot(4,1,4)
plot(t, x(4,1:end-1))
ylabel('dY, m/s')
xlabel('Time, s')

figure(3)
title('Original control input and modified input')
subplot(2,1,1)
plot(t, uopt(1,:), 'b', t, kxopt(1,:), 'r')
ylabel('u1, m/s^2')
legend({'Optimized', 'Original'})
subplot(2,1,2)
plot(t, uopt(2,:), 'b', t, kxopt(2,:), 'r')
ylabel('u2, m/s^2')
legend({'Optimized', 'Original'})
xlabel('Time, s')

%%




