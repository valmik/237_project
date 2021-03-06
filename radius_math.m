clear
close all
clc



%% Radial CBF

syms x y dx dy u1 u2 real

q = [x; y];
dq = [dx; dy];
u = [u1; u2];
z = [q; dq];
dz = [dq; u];

r = sqrt(x^2 + y^2 + 0.00001);
dr = jacobian(r, q)*dq;

% h = -(r + dr - 1);
h = -(r + 0.5*dr^2 - 1);

f = [zeros(2), eye(2); zeros(2), zeros(2)]*z;
g = [zeros(2); eye(2)]*u;

Lfh = jacobian(h, z)*f;
Lgh = jacobian(h, z)*g;

%% Velocity CBF

syms ts real

h2 = sqrt(dx^2 + dy^2)*ts - 1;
Lfh2 = jacobian(h2, z)*f;
Lgh2 = jacobian(h2, z)*g;


%% Generate functions

if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(f, 'File', 'gen/f_gen', 'Vars', {z});
matlabFunction(g, 'File', 'gen/g_gen', 'Vars', {z, u});
matlabFunction(h, 'File', 'gen/h_gen', 'Vars', {z});
matlabFunction(Lfh, 'File', 'gen/Lfh_gen', 'Vars', {z});
matlabFunction(Lgh, 'File', 'gen/Lgh_gen', 'Vars', {z, u});

matlabFunction(h2, 'File', 'gen/h2_gen', 'Vars', {z, ts});
matlabFunction(Lfh2, 'File', 'gen/Lfh2_gen', 'Vars', {z, ts});
matlabFunction(Lgh2, 'File', 'gen/Lgh2_gen', 'Vars', {z, u, ts});

%%

test = 0.1:0.1:2;
check = zeros(size(test));
for index = 1:numel(test)
    check(index) = subs(h, z, [test(index); 0;0.1;0]);
end

plot(test, check)



















