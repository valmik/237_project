clear
close all
clc

%% Linearization
syms x y vel theta u1 u2 real

dx = vel*cos(theta);
dy = vel*sin(theta);

% v1 = 0.5*cos(theta)*u1 - 0.5*vel*sin(theta)*u2;
% v2 = 0.5*sin(theta)*u1 + 0.5*vel*cos(theta)*u2;


%% Radial CBF


q = [x; y];
dq = [dx; dy];
% v = [v1; v2];
% z = [q; dq];
% dz = [dq; v];

r = sqrt(x^2 + y^2 + 0.00001);
dr = jacobian(r, q)*dq;

% h = -(r + dr - 1);
h = -(r + 0.5*dr^2 - 1);

% f = [zeros(2), eye(2); zeros(2), zeros(2)]*z;
% g = [zeros(2); eye(2)]*v;
% 
% Lfh = jacobian(h, z)*f;
% Lgh = jacobian(h, z)*g;

%% 

z = [x; y; vel; theta];
u = [u1; u2];

f = [vel*cos(theta); vel*sin(theta); 0; 0];
g = [zeros(2); eye(2)]*u;

Lfh = jacobian(h, z)*f;
Lgh = jacobian(h, z)*g;


%% Velocity cbf

h2 = vel^2 - 0.0001;
Lfh2 = jacobian(h2, z)*f;
Lgh2 = jacobian(h2, z)*g;


%% Generate functions

if ~exist('./gen2')
    mkdir('./gen2')
end
addpath('./gen2')

matlabFunction(f, 'File', 'gen2/f_gen', 'Vars', {z});
matlabFunction(g, 'File', 'gen2/g_gen', 'Vars', {z, u});
matlabFunction(h, 'File', 'gen2/h_gen', 'Vars', {z});
matlabFunction(Lfh, 'File', 'gen2/Lfh_gen', 'Vars', {z});
matlabFunction(Lgh, 'File', 'gen2/Lgh_gen', 'Vars', {z, u});

matlabFunction(h2, 'File', 'gen2/h2_gen', 'Vars', {z});
matlabFunction(Lfh2, 'File', 'gen2/Lfh2_gen', 'Vars', {z});
matlabFunction(Lgh2, 'File', 'gen2/Lgh2_gen', 'Vars', {z, u});
