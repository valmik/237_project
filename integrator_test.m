clear all
close all
clc

q0 = [-5; 0];
q_goal = [2; 0];

N = 150;

[q_opt, u_opt] = integrator_MPC(N, q0, q_goal, 0.05);

plot(q_opt(1,:), q_opt(2,:))




