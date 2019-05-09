%% Playing with chained form systems

lie = @(f, g, x) jacobian(g, x)*f;

%% First order chained form system

syms x1 x2 x3 x4
x = [x1;x2;x3;x4];

f = [x4; 0; x2*x4; 0];
g = [[0;0;0;1], [0;1;0;0]];



[g, lie_br(f, g, x)]




