
syms x y dx dy u1 u2 real

q = [x; y];
dq = [dx; dy];
u = [u1; u2];
z = [q; dq];
dz = [dq; u];

r = sqrt(x^2 + y^2);
dr = jacobian(r, q)*dq;

h = r + dr - 1;

f = [zeros(2), eye(2); zeros(2), zeros(2)]*z;
g = [zeros(2); eye(2)]*u;

Lfh = jacobian(h, z)*f;
Lgh = jacobian(h, z)*g;














